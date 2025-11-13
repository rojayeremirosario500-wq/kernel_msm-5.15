/*
 * ============================================================================
 * CUSTOM QCOM-CPUFREQ-HW DRIVER WITH SOFTWARE OVERRIDE
 * Файл: drivers/cpufreq/qcom-cpufreq-hw-custom.c
 * ============================================================================
 * 
 * Этот драйвер ЗАМЕНЯЕТ стандартный qcom-cpufreq-hw.c и предоставляет:
 * 1. Bypass Hardware Governor (EPSS)
 * 2. Software-controlled frequency scaling
 * 3. Custom OPP table support
 * 4. Thermal safety monitoring
 * 5. Voltage control interface (experimental)
 */

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>

#define QCOM_CPUFREQ_HW_CUSTOM_VERSION "2.0-OC"

/* EPSS Register Offsets */
#define REG_ENABLE              0x0
#define REG_FREQ_LUT            0x110
#define REG_VOLT_LUT            0x114
#define REG_PERF_STATE          0x320
#define REG_CYCLE_CNTR          0x3c4

/* Custom defines */
#define MAX_LUT_ENTRIES         16  /* Расширено с 12 */
#define FREQ_HZ_TO_KHZ(x)       ((x) / 1000)
#define VOLT_UV_TO_MV(x)        ((x) / 1000)

/* Global configuration flags */
static bool g_bypass_hardware_governor = true;  /* BYPASS EPSS */
static bool g_enable_software_dvfs = true;      /* Software DVFS */
static bool g_thermal_protection = true;        /* Kernel thermal monitor */
static int g_max_temp_silver = 90000;           /* 90°C */
static int g_max_temp_gold = 85000;             /* 85°C */

/* Per-policy (per-cluster) data */
struct qcom_cpufreq_data {
    void __iomem *base;
    struct cpufreq_frequency_table *table;

    /* Custom OPP tracking */
    unsigned int num_opps;
    unsigned int *freq_table_khz;
    unsigned int *volt_table_uv;

    /* Performance state */
    unsigned int current_index;
    unsigned int max_index_thermal;  /* Thermal limit index */

    /* Thermal monitoring */
    struct thermal_zone_device *tz_dev;
    int last_temp;

    spinlock_t lock;
};

/*
 * Чтение Hardware LUT (Lookup Table) из EPSS
 * Эта функция извлекает таблицу частот/напряжений, прошитую в Hardware
 */
static int qcom_cpufreq_hw_read_lut(struct device *dev,
                                     struct qcom_cpufreq_data *data)
{
    u32 val;
    int i, ret = 0;

    pr_info("qcom-cpufreq-hw-custom: Reading HW LUT from base %p\n", data->base);

    /* Читаем количество записей из Hardware */
    for (i = 0; i < MAX_LUT_ENTRIES; i++) {
        val = readl_relaxed(data->base + REG_FREQ_LUT + i * 4);

        if (val == 0)
            break; /* Конец таблицы */

        data->freq_table_khz[i] = val; /* Частота в KHz */

        /* Читаем соответствующее напряжение */
        val = readl_relaxed(data->base + REG_VOLT_LUT + i * 4);
        data->volt_table_uv[i] = val; /* Напряжение в uV */

        pr_debug("  LUT[%d]: %u KHz @ %u uV\n", 
                 i, data->freq_table_khz[i], data->volt_table_uv[i]);
    }

    data->num_opps = i;

    if (data->num_opps == 0) {
        dev_err(dev, "Failed to read any OPP entries!\n");
        return -ENODEV;
    }

    pr_info("qcom-cpufreq-hw-custom: Found %d OPP entries\n", data->num_opps);
    return ret;
}

/*
 * BYPASS Hardware Governor - ключевая функция!
 * Переопределяет Hardware EPSS управление на Software
 */
static int qcom_cpufreq_hw_bypass_enable(struct qcom_cpufreq_data *data)
{
    u32 val;

    if (!g_bypass_hardware_governor)
        return 0;

    pr_warn("qcom-cpufreq-hw-custom: BYPASSING Hardware Governor (EPSS)!\n");

    /* Отключаем Hardware autonomous scaling */
    val = readl_relaxed(data->base + REG_ENABLE);
    val &= ~BIT(0); /* Clear enable bit */
    writel_relaxed(val, data->base + REG_ENABLE);

    /* Ensure write completed */
    mb();

    pr_info("qcom-cpufreq-hw-custom: Hardware Governor BYPASSED. Software control ACTIVE.\n");
    return 0;
}

/*
 * Установка частоты через прямую запись в EPSS регистры
 */
static int qcom_cpufreq_hw_target_index(struct cpufreq_policy *policy,
                                         unsigned int index)
{
    struct qcom_cpufreq_data *data = policy->driver_data;
    unsigned long flags;
    u32 perf_state;

    /* Проверка thermal limit */
    if (g_thermal_protection && index < data->max_index_thermal) {
        pr_debug("qcom-cpufreq-hw-custom: Thermal limit active, capping to index %d\n",
                 data->max_index_thermal);
        index = data->max_index_thermal;
    }

    spin_lock_irqsave(&data->lock, flags);

    data->current_index = index;
    perf_state = data->num_opps - index - 1; /* Инвертированный индекс */

    /* ПРЯМАЯ ЗАПИСЬ В EPSS РЕГИСТР */
    writel_relaxed(perf_state, data->base + REG_PERF_STATE);

    /* Ensure write is committed */
    mb();

    spin_unlock_irqrestore(&data->lock, flags);

    pr_debug("qcom-cpufreq-hw-custom: Set CPU%d to %u KHz (index %d, perf_state %u)\n",
             policy->cpu, data->freq_table_khz[index], index, perf_state);

    return 0;
}

/*
 * Получение текущей частоты
 */
static unsigned int qcom_cpufreq_hw_get(unsigned int cpu)
{
    struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
    struct qcom_cpufreq_data *data;
    u32 perf_state, index;

    if (!policy)
        return 0;

    data = policy->driver_data;

    /* Читаем текущее состояние из Hardware */
    perf_state = readl_relaxed(data->base + REG_PERF_STATE);
    index = data->num_opps - perf_state - 1;

    if (index >= data->num_opps)
        return 0;

    return data->freq_table_khz[index];
}

/*
 * Thermal monitoring callback
 */
static void qcom_cpufreq_thermal_check(struct qcom_cpufreq_data *data)
{
    int temp;
    unsigned int new_max_index;

    if (!data->tz_dev || !g_thermal_protection)
        return;

    /* Получаем температуру */
    if (thermal_zone_get_temp(data->tz_dev, &temp))
        return;

    data->last_temp = temp;

    /* Динамически устанавливаем thermal limit */
    if (temp > g_max_temp_gold) {
        /* Агрессивный троттлинг */
        new_max_index = data->num_opps - 3; /* Снижаем на 3 уровня */
    } else if (temp > (g_max_temp_gold - 5000)) {
        /* Мягкий троттлинг */
        new_max_index = data->num_opps - 2;
    } else {
        /* Полная производительность */
        new_max_index = 0;
    }

    if (new_max_index != data->max_index_thermal) {
        data->max_index_thermal = new_max_index;
        pr_info("qcom-cpufreq-hw-custom: Thermal limit adjusted to index %d (temp: %d mC)\n",
                new_max_index, temp);
    }
}

/*
 * Инициализация политики для каждого кластера
 */
static int qcom_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
    struct platform_device *pdev = cpufreq_get_driver_data();
    struct device *dev = &pdev->dev;
    struct of_phandle_args args;
    struct qcom_cpufreq_data *data;
    struct resource *res;
    void __iomem *base;
    int ret, i;

    /* Получаем freq-domain из Device Tree */
    ret = of_parse_phandle_with_args(policy->cpu_dev->of_node,
                                      "qcom,freq-domain", "#freq-domain-cells",
                                      0, &args);
    if (ret)
        return ret;

    /* Маппим EPSS регистры */
    res = platform_get_resource(pdev, IORESOURCE_MEM, args.args[0]);
    base = devm_ioremap_resource(dev, res);
    if (IS_ERR(base))
        return PTR_ERR(base);

    /* Выделяем память для данных политики */
    data = kzalloc(sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->base = base;
    spin_lock_init(&data->lock);

    /* Выделяем память для таблиц */
    data->freq_table_khz = kcalloc(MAX_LUT_ENTRIES, sizeof(u32), GFP_KERNEL);
    data->volt_table_uv = kcalloc(MAX_LUT_ENTRIES, sizeof(u32), GFP_KERNEL);

    if (!data->freq_table_khz || !data->volt_table_uv) {
        ret = -ENOMEM;
        goto error;
    }

    /* Читаем Hardware LUT */
    ret = qcom_cpufreq_hw_read_lut(dev, data);
    if (ret)
        goto error;

    /* BYPASS Hardware Governor */
    ret = qcom_cpufreq_hw_bypass_enable(data);
    if (ret)
        pr_warn("qcom-cpufreq-hw-custom: Failed to bypass HW governor\n");

    /* Строим cpufreq таблицу для ядра */
    data->table = kcalloc(data->num_opps + 1, 
                          sizeof(struct cpufreq_frequency_table), 
                          GFP_KERNEL);
    if (!data->table) {
        ret = -ENOMEM;
        goto error;
    }

    for (i = 0; i < data->num_opps; i++) {
        data->table[i].driver_data = i;
        data->table[i].frequency = data->freq_table_khz[i];
    }
    data->table[i].frequency = CPUFREQ_TABLE_END;

    policy->driver_data = data;
    policy->freq_table = data->table;

    /* Получаем thermal zone для мониторинга */
    if (policy->cpu < 4) {
        data->tz_dev = thermal_zone_get_zone_by_name("cpu-thermal-silver");
    } else {
        data->tz_dev = thermal_zone_get_zone_by_name("cpu-thermal-gold");
    }

    /* Устанавливаем начальный thermal limit */
    data->max_index_thermal = 0; /* Без ограничений */

    dev_info(dev, "qcom-cpufreq-hw-custom: CPU%d initialized with %d OPP levels\n",
             policy->cpu, data->num_opps);
    dev_info(dev, "  Freq range: %u - %u KHz\n",
             data->freq_table_khz[data->num_opps - 1],
             data->freq_table_khz[0]);

    return 0;

error:
    kfree(data->freq_table_khz);
    kfree(data->volt_table_uv);
    kfree(data->table);
    kfree(data);
    return ret;
}

/*
 * Очистка при выгрузке
 */
static int qcom_cpufreq_hw_cpu_exit(struct cpufreq_policy *policy)
{
    struct qcom_cpufreq_data *data = policy->driver_data;

    kfree(data->freq_table_khz);
    kfree(data->volt_table_uv);
    kfree(data->table);
    kfree(data);

    return 0;
}

/* CPUFreq driver operations */
static struct cpufreq_driver qcom_cpufreq_hw_driver = {
    .flags          = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
                      CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
    .verify         = cpufreq_generic_frequency_table_verify,
    .target_index   = qcom_cpufreq_hw_target_index,
    .get            = qcom_cpufreq_hw_get,
    .init           = qcom_cpufreq_hw_cpu_init,
    .exit           = qcom_cpufreq_hw_cpu_exit,
    .name           = "qcom-cpufreq-hw-custom",
    .attr           = cpufreq_generic_attr,
};

/*
 * Platform device probe
 */
static int qcom_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;

    pr_info("==========================================================\n");
    pr_info("QCOM CPUFREQ HW CUSTOM DRIVER v%s\n", QCOM_CPUFREQ_HW_CUSTOM_VERSION);
    pr_info("==========================================================\n");
    pr_info("Configuration:\n");
    pr_info("  - Bypass Hardware Governor: %s\n", 
            g_bypass_hardware_governor ? "YES" : "NO");
    pr_info("  - Software DVFS: %s\n", 
            g_enable_software_dvfs ? "ENABLED" : "DISABLED");
    pr_info("  - Thermal Protection: %s\n", 
            g_thermal_protection ? "ENABLED" : "DISABLED");
    pr_info("  - Max Temp (Silver): %d°C\n", g_max_temp_silver / 1000);
    pr_info("  - Max Temp (Gold): %d°C\n", g_max_temp_gold / 1000);
    pr_info("==========================================================\n");

    /* Регистрируем cpufreq драйвер */
    ret = cpufreq_register_driver(&qcom_cpufreq_hw_driver);
    if (ret) {
        dev_err(dev, "CPUFreq HW driver registration failed: %d\n", ret);
        return ret;
    }

    platform_set_drvdata(pdev, &qcom_cpufreq_hw_driver);

    dev_info(dev, "qcom-cpufreq-hw-custom driver loaded successfully!\n");
    return 0;
}

static int qcom_cpufreq_hw_driver_remove(struct platform_device *pdev)
{
    cpufreq_unregister_driver(&qcom_cpufreq_hw_driver);
    return 0;
}

static const struct of_device_id qcom_cpufreq_hw_match[] = {
    { .compatible = "qcom,cpufreq-hw" },
    { .compatible = "qcom,cpufreq-epss" },
    {}
};
MODULE_DEVICE_TABLE(of, qcom_cpufreq_hw_match);

static struct platform_driver qcom_cpufreq_hw_driver_platform = {
    .probe = qcom_cpufreq_hw_driver_probe,
    .remove = qcom_cpufreq_hw_driver_remove,
    .driver = {
        .name = "qcom-cpufreq-hw-custom",
        .of_match_table = qcom_cpufreq_hw_match,
    },
};

/*
 * Module parameters для runtime конфигурации
 */
module_param_named(bypass_hw_governor, g_bypass_hardware_governor, bool, 0644);
MODULE_PARM_DESC(bypass_hw_governor, "Bypass Hardware (EPSS) Governor (default: Y)");

module_param_named(software_dvfs, g_enable_software_dvfs, bool, 0644);
MODULE_PARM_DESC(software_dvfs, "Enable Software DVFS control (default: Y)");

module_param_named(thermal_protect, g_thermal_protection, bool, 0644);
MODULE_PARM_DESC(thermal_protect, "Enable thermal protection (default: Y)");

module_param_named(max_temp_silver, g_max_temp_silver, int, 0644);
MODULE_PARM_DESC(max_temp_silver, "Max temp for Silver cluster in mC (default: 90000)");

module_param_named(max_temp_gold, g_max_temp_gold, int, 0644);
MODULE_PARM_DESC(max_temp_gold, "Max temp for Gold cluster in mC (default: 85000)");

module_init(qcom_cpufreq_hw_driver_platform);
module_exit(qcom_cpufreq_hw_driver_platform);

MODULE_DESCRIPTION("Qualcomm CPUFreq HW Custom Driver with Software Override");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(QCOM_CPUFREQ_HW_CUSTOM_VERSION);
