
#include <linux/bitops.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

// 定义了输入模式的枚举类型，用于指定面板的工作模式。
enum modi_input {
	MODI_INPUT_PRGB_THROUGH = 0x0, // 24位并行RGB直通模式
	MODI_INPUT_PRGB_ALIGNED = 0x1, // 24位并行RGB对齐模式
	MODI_INPUT_UNKNOWN = 0x2,      // 未知输入模式
};

// 定义了一个字符串数组，用于描述不同的输入模式。注意这里的描述可能并不完全对应上面的枚举值。
static const char * const modipanel_inputs[] = {
	"24 bit parallel RGB through", // 对应MODI_INPUT_PRGB_THROUGH
	"24 bit parallel RGB aligned", // 对应MODI_INPUT_PRGB_ALIGNED
	"unknow input mode"
};

// 定义了一个结构体，用于存储面板的配置信息。
struct modipanel_config {
	u32 width_mm; // 面板宽度（毫米）
	u32 height_mm; // 面板高度（毫米）
	bool flip_horizontal; // 是否水平翻转
	bool flip_vertical; // 是否垂直翻转
	enum modi_input input; // 输入模式
	u32 vreg1out_mv; // 输出电压
	u32 vcom_high_percent; // VCOM高电平百分比
	u32 vcom_amplitude_percent; // VCOM幅度百分比
	bool dclk_active_high; // DCLK是否为高电平有效
	bool de_active_high; // DE信号是否为高电平有效
	bool hsync_active_high; // HSYNC信号是否为高电平有效
	bool vsync_active_high; // VSYNC信号是否为高电平有效
	u8 syncmode; // 同步模式
	u8 gamma_corr_pos[8]; // 正伽马校正系数
	u8 gamma_corr_neg[8]; // 负伽马校正系数
};

// 定义了一个结构体，包含设备相关信息和配置参数。
struct modipanel {
	struct device *dev; // 设备指针
	const struct modipanel_config *conf; // 配置指针
	struct drm_panel panel; // DRM面板结构体
	struct regmap *regmap; // 寄存器映射
	struct regulator_bulk_data supplies[3]; // 电源管理相关数据
	struct gpio_desc *reset_gpio; // GPIO复位引脚
	struct gpio_desc *cs_gpio;
	enum modi_input input; // 输入模式
	struct videomode vm; // 视频模式
	u8 gamma[8]; // 伽马值
	u8 vreg1out; // 输出电压设置
	u8 vcom_high; // VCOM高电平设置
	u8 vcom_amplitude; // VCOM幅度设置
};

/**
 * 将drm_panel结构体转换为其容器modipanel结构体的辅助函数。
 */
static inline struct modipanel *panel_to_modi(struct drm_panel *panel)
{
    return container_of(panel, struct modipanel, panel);
}

/**
 * SPI写入函数，用于向modi控制器发送数据。
 * @context: 设备上下文，通常是指向设备的指针
 * @data: 要写入的数据
 * @count: 数据长度
 * 返回值：成功返回0，失败返回错误码
 */
#if 0 // native
static int modi_regmap_spi_write(void *context, const void *data,
                                    size_t count)
{
    struct device *dev = context; // 获取设备上下文
    struct spi_device *spi = to_spi_device(dev); // 获取SPI设备
    u8 buf[2]; // 缓冲区用于存储要发送的数据

    memcpy(buf, data, 2); // 复制数据到缓冲区
    buf[0] &= ~0x80; // 清除最高位以指示这是一个写操作

    //dev_dbg(dev, "WRITE: %02x %02x\n", buf[0], buf[1]); // 调试信息
	printk("WL---> WRITE: %02x %02x\n", buf[0], buf[1]);
    return spi_write_then_read(spi, buf, 2, NULL, 0); // 执行SPI写操作
}
#else // new

#define MAX_WRITE_SIZE 64 // 假设最大写入数据长度为 64 字节
static int modi_regmap_spi_write(void *context, const void *data,
                                 size_t count)
{
    struct device *dev = context; // 获取设备上下文
    struct spi_device *spi = to_spi_device(dev); // 获取SPI设备
    u8 buf[MAX_WRITE_SIZE]; // 缓冲区用于存储要发送的数据
    int ret;

    // 检查参数有效性
    if (!spi || !data || count == 0 || count > sizeof(buf)) {
        printk("Invalid parameters for SPI write\n");
        return -EINVAL;
    }

    // 复制数据到缓冲区
    memcpy(buf, data, count);

    // 清除写操作标志位（如果需要）
    if (count >= 1) { // 假设仅对第一个字节清除最高位
        buf[0] &= ~0x80;
    }

    // 打印调试信息
    printk("WL---> WRITE: REG = 0x%02x, VAL: ", buf[0]);
    for (int i = 1; i < count; i++) {
        printk("0x%02x ", buf[i]);
    }

	printk("\n");
    // 执行SPI写操作
    ret = spi_write_then_read(spi, buf, count, NULL, 0);
    if (ret < 0) {
        dev_err(dev, "SPI write failed: %d\n", ret);
        return ret;
    }

    return 0; // 成功返回 0
}
#endif

/**
 * SPI读取函数，用于从控制器读取数据。
 * @context: 设备上下文
 * @reg: 要读取的寄存器地址
 * @reg_size: 寄存器地址大小
 * @val: 存储读取值的缓冲区
 * @val_size: 值的大小
 * 返回值：成功返回0，失败返回错误码
 */
static int modi_regmap_spi_read(void *context, const void *reg,
                                   size_t reg_size, void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u8 buf[1];
	//u8 rval[1] = {0}; // 初始化值

	memcpy(buf, reg, 1); // 复制寄存器地址到缓冲区
	buf[0] |= 0x80; // 设置最高位以指示这是一个读操作

	// dev_dbg(dev, "READ: %02x reg size = %zu, val size = %zu\n",
	// 	buf[0], reg_size, val_size);
	printk("WL---> READ: %02x reg size = %zu, val size = %zu\n",
		buf[0], reg_size, val_size);
	return spi_write_then_read(spi, buf, 1, val, 1); // 执行SPI读操作
	//spi_write_then_read(spi, buf, 1, (void*)rval, 1); // 执行SPI读操作
	//printk("WL---> READ, REG:0x%02x/0x%02x, VAL: 0x%02x\n", buf[0], buf[0]&0x7f, rval[0]);
	//return 0;
}

/**
 * 定义一个regmap_bus结构体实例，用于注册regmap总线操作函数。
 */
static struct regmap_bus modipanel_regmap_bus = {
	.write = modi_regmap_spi_write, // 写函数
	.read = modi_regmap_spi_read,   // 读函数
	.reg_format_endian_default = REGMAP_ENDIAN_BIG, // 寄存器格式默认大端
	.val_format_endian_default = REGMAP_ENDIAN_BIG, // 值格式默认大端
};

/**
 * 判断寄存器是否是易失性的回调函数。
 */
static bool modi_volatile_reg(struct device *dev, unsigned int reg)
{
    	return false; // 默认所有寄存器都不是易失性
}

/**
 * 判断寄存器是否可写的回调函数。
 */
static bool modi_writeable_reg(struct device *dev, unsigned int reg)
{
	if (reg == 0x00) // 寄存器0不可写
		return false;
	return true;
}

/**
 * 配置regmap的相关属性。
 */
static const struct regmap_config modipanel_regmap_config = {
    .reg_bits = 8, // 每个寄存器8位
    .val_bits = 8, // 每个值8位
    .max_register = 0xff, // 最大寄存器地址
    .cache_type = REGCACHE_RBTREE, // 使用红黑树缓存
    .volatile_reg = modi_volatile_reg, // 易失性寄存器判断函数
    .writeable_reg = modi_writeable_reg, // 可写寄存器判断函数
};


static int st7701s_spi_write_cmd_9bit
(struct spi_device *spi, u8 cmd){
    u16 tx9;  // 9-bit 数据, 占用16位变量
    int ret;

    // bit8=0 (DC=0), bit[7:0] = cmd
    tx9 = (0 << 8) | cmd;

    // spi_write/spi_sync_* 都可. 如果控制器支持9bit，会一次发送9bit
    ret = spi_write(spi, &tx9, sizeof(tx9));
    if (ret < 0)
        printk("WL ---> Failed 9bit-cmd=0x%02X\n", cmd);
    return ret;
}

/* 发送任意长度的数据：每个字节都要加 DC=1 */
static int st7701s_spi_write_data_9bit
(struct spi_device *spi,const u8 *buf, size_t len){

    int i, ret;
    // 为了简化，这里用一个临时 9-bit 数组再一次性发
    // 或者可以循环 spi_write 逐字节
    // 简易做法：开一个临时数组 (len * sizeof(u16))
    u16 *txbuf = kzalloc(len * sizeof(u16), GFP_KERNEL);
    if (!txbuf)
        return -ENOMEM;

    for (i = 0; i < len; i++) {
        txbuf[i] = (1 << 8) | buf[i];  // DC=1 + 数据
    }

    ret = spi_write(spi, txbuf, len * sizeof(u16));
    if (ret < 0)
        printk("WL---> Failed 9bit-data write, len=%zu\n", len);

    kfree(txbuf);
    return ret;
}

/*
先发送一个读命令（9 位），然后通过 MISO 接收数据。
数据在 SCL 的上升沿采样，MISO 输出数据在下降沿同步。
*/
static int st7701s_spi_read_data_9bit
(struct spi_device *spi, u8 cmd, u8 *rxbuf, size_t rxlen){

    struct spi_transfer xfers[2];
    struct spi_message msg;
    u16 cmd9, *txbuf;
    int i, ret;

    /* 初始化 SPI 消息 */
    spi_message_init(&msg);

    /* 第一段：发送读命令 */
    cmd9 = (0 << 8) | cmd; /* bit[8]=0 表示命令 */
    memset(&xfers[0], 0, sizeof(xfers[0]));
    xfers[0].tx_buf = &cmd9;
    xfers[0].len = sizeof(cmd9);
    spi_message_add_tail(&xfers[0], &msg);

    /* 第二段：接收数据 */
    txbuf = kzalloc(rxlen * sizeof(u16), GFP_KERNEL);
    if (!txbuf)
        return -ENOMEM;

    for (i = 0; i < rxlen; i++) {
        txbuf[i] = (1 << 8) | 0x00; /* bit[8]=1 表示数据 */
    }

    memset(&xfers[1], 0, sizeof(xfers[1]));
    xfers[1].tx_buf = txbuf;
    xfers[1].rx_buf = txbuf;
    xfers[1].len = rxlen * sizeof(u16);
    spi_message_add_tail(&xfers[1], &msg);

    /* 执行 SPI 传输 */
    ret = spi_sync(spi, &msg);
    if (ret < 0) {
        printk("WANGLEI------> SPI read failed: %d\n", ret);
        goto out;
    }

    /* 提取接收到的数据 */
    for (i = 0; i < rxlen; i++) {
        rxbuf[i] = txbuf[i] & 0xFF; /* 只取低 8 位 */
    }

out:
    kfree(txbuf);
    return ret;
}

static int spi_write_command_and_data
(struct spi_device *spi, u8 cmd, const u8 *data, size_t len){

    int ret;

    // 写入命令
    ret = st7701s_spi_write_cmd_9bit(spi, cmd);
    if (ret < 0) {
        printk("WANGLEI------> spi write error: cmd=0x%02x, line=%d\n", cmd, __LINE__);
        return ret;
    }

	if (NULL == data) return 0;

    // 写入数据
    ret = st7701s_spi_write_data_9bit(spi, data, len);
    if (ret < 0) {
        printk("WANGLEI------> spi write error: data=%p, size=%zu, line=%d\n", data, len, __LINE__);
        return ret;
    }

    return 0;
}

static int spi_read_id(struct spi_device *spi){

	int ret;
	u8 idbuf[3] = {0};
	memset(idbuf, 0, sizeof(idbuf));
	ret = st7701s_spi_read_data_9bit(spi, 0x04, idbuf, 3);
	if (ret < 0) {
		printk("WANGLEI------> Failed read ID\n");
		return ret;
	}

	printk("WANGLEI------> ID: 0x%02x 0x%02x 0x%02x\n", idbuf[0], idbuf[1], idbuf[2]);
	return 0;
}

static int modi_init
(struct drm_panel *panel, struct modipanel *modi){

	struct spi_device *spi = to_spi_device(modi->dev);
	
	u8 data0[] = {0x77, 0x01, 0x00, 0x00, 0x13};
	spi_write_command_and_data(spi, 0xFF, data0, sizeof(data0));

	u8 data1[] = {0x08};
	spi_write_command_and_data(spi, 0xEF, data1, sizeof(data1));

	u8 data2[] = {0x77, 0x01, 0x00, 0x00, 0x10};
	spi_write_command_and_data(spi, 0xFF, data2, sizeof(data2));

	u8 data3[] = {0x63, 0x00};
	spi_write_command_and_data(spi, 0xC0, data3, sizeof(data3));

	u8 data4[] = {0x14, 0x0C};
	spi_write_command_and_data(spi, 0xC1, data4, sizeof(data4));

	u8 data5[] = {0x37, 0x02};
	spi_write_command_and_data(spi, 0xC2, data5, sizeof(data5));

	//增加屏厂优化参数：
	u8 data_add01[] = {0x02};
	spi_write_command_and_data(spi, 0xC3, data_add01, sizeof(data_add01));

	u8 data6[] = {0x10};
	spi_write_command_and_data(spi, 0xCC, data6, sizeof(data6));

	u8 data7[] = {0x06, 0x10, 0x16, 0x0D, 0x11, 0x06, 0x08, 0x07, 
					0x08, 0x22, 0x04, 0x14, 0x0F, 0x29, 0x2F, 0x1F};
	spi_write_command_and_data(spi, 0xB0, data7, sizeof(data7));

	u8 data8[] = {0x0F, 0x18, 0x1E, 0x0C, 0x0F, 0x06, 0x08, 0x0A, 
					0x09, 0x24, 0x05, 0x10, 0x11, 0x2A, 0x34, 0x1F};
	spi_write_command_and_data(spi, 0xB1, data8, sizeof(data8));

	u8 data9[] = {0x77, 0x01, 0x00, 0x00, 0x11};
	spi_write_command_and_data(spi, 0xFF, data9, sizeof(data9));

	u8 data10[] = {0x2D};//0X4D
	spi_write_command_and_data(spi, 0xB0, data10, sizeof(data10));

	u8 data11[] = {0x4D};
	spi_write_command_and_data(spi, 0xB1, data11, sizeof(data11));

	u8 data12[] = {0x81};
	spi_write_command_and_data(spi, 0xB2, data12, sizeof(data12));

	u8 data13[] = {0x80};
	spi_write_command_and_data(spi, 0xB3, data13, sizeof(data13));

	u8 data14[] = {0x4E};
	spi_write_command_and_data(spi, 0xB5, data14, sizeof(data14));

	u8 data15[] = {0x85};
	spi_write_command_and_data(spi, 0xB7, data15, sizeof(data15));

	u8 data16[] = {0x32};
	spi_write_command_and_data(spi, 0xB8, data16, sizeof(data16));

	u8 data17[] = {0x03};
	spi_write_command_and_data(spi, 0xBB, data17, sizeof(data17));

	u8 data18[] = {0x08};
	spi_write_command_and_data(spi, 0xC1, data18, sizeof(data18));

	u8 data19[] = {0x08};
	spi_write_command_and_data(spi, 0xC2, data19, sizeof(data19));

	u8 data20[] = {0x88};
	spi_write_command_and_data(spi, 0xD0, data20, sizeof(data20));

	u8 data21[] = {0x00, 0x00, 0x02};
	spi_write_command_and_data(spi, 0xE0, data21, sizeof(data21));

	u8 data22[] = {0x06, 0x28, 0x08, 0x28, 0x05, 0x28, 0x07, 0x28, 0x0E, 0x33, 0x33};
	spi_write_command_and_data(spi, 0xE1, data22, sizeof(data22));

	u8 data23[] = {0x30, 0x30, 0x33, 0x33, 0x34, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00};
	spi_write_command_and_data(spi, 0xE2, data23, sizeof(data23));

	u8 data24[] = {0x00, 0x00, 0x33, 0x33};
	spi_write_command_and_data(spi, 0xE3, data24, sizeof(data24));

	u8 data25[] = {0x44, 0x44};
	spi_write_command_and_data(spi, 0xE4, data25, sizeof(data25));

	u8 data26[] = {0x09, 0x2F, 0x2C, 0x8C, 0x0B, 0x31, 0x2C, 0x8C, 
					0x0D, 0x33, 0x2C, 0x8C, 0x0F, 0x35, 0x2C, 0x8C};
	spi_write_command_and_data(spi, 0xE5, data26, sizeof(data26));

	u8 data27[] = {0x00, 0x00, 0x33, 0x33};
	spi_write_command_and_data(spi, 0xE6, data27, sizeof(data27));

	u8 data28[] = {0x44, 0x44};
	spi_write_command_and_data(spi, 0xE7, data28, sizeof(data28));

	u8 data29[] = {0x08, 0x2E, 0x2C, 0x8C, 0x0A, 0x30, 0x2C, 0x8C, 
					0x0C, 0x32, 0x2C, 0x8C, 0x0E, 0x34, 0x2C, 0x8C};
	spi_write_command_and_data(spi, 0xE8, data29, sizeof(data29));

	u8 data30[] = {0x36, 0x00};
	spi_write_command_and_data(spi, 0xE9, data30, sizeof(data30));

	u8 data31[] = {0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40};
	spi_write_command_and_data(spi, 0xEB, data31, sizeof(data31));

	u8 data32[] = {0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF, 
					0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF};
	spi_write_command_and_data(spi, 0xED, data32, sizeof(data32));

	u8 data33[] = {0x08, 0x08, 0x08, 0x45, 0x3F, 0x54};
	spi_write_command_and_data(spi, 0xEF, data33, sizeof(data33));

	u8 data34[] = {0x77, 0x01, 0x00, 0x00, 0x13};
	spi_write_command_and_data(spi, 0xFF, data34, sizeof(data34));

	u8 data35[] = {0x00, 0x0E};
	spi_write_command_and_data(spi, 0xE8, data35, sizeof(data35));

	// SPI_WriteComm(0x11);
	spi_write_command_and_data(spi, 0x11, NULL, 0); // soft_reset.
	msleep(120);

	u8 data36[] = {0x00, 0x0C};
	spi_write_command_and_data(spi, 0xE8, data36, sizeof(data36));
	msleep(10);

	u8 data37[] = {0x00, 0x00};
	spi_write_command_and_data(spi, 0xE8, data37, sizeof(data37));

	u8 data38[] = {0x77, 0x01, 0x00, 0x00, 0x00};
	spi_write_command_and_data(spi, 0xFF, data38, sizeof(data38));

	u8 data39[] = {0x00};
	spi_write_command_and_data(spi, 0x36, data39, sizeof(data39)); // rgb format.

	// SPI_WriteComm(0x29);
	spi_write_command_and_data(spi, 0x29, NULL, 0); // screen on.
	msleep(120);

	u8 data40[] = {0x77, 0x01, 0x00, 0x00, 0x10};
	spi_write_command_and_data(spi, 0xFF, data40, sizeof(data40));

	u8 data41[] = {0x00};
	spi_write_command_and_data(spi, 0xE5, data41, sizeof(data41));

	return 0;
}

/**
 * 面板电源开启函数，初始化面板电源。
 * @modi: 指向modipanel结构的指针。
 * 返回值：成功返回0，失败返回错误码。
 */
static int modi_power_on(struct modipanel *modi)
{
	//int ret;

	/* 设置RESET引脚为高电平 */
	//gpiod_set_value(modi->reset_gpio, 1);

	/* 启动电源调节器 */
	// ret = regulator_bulk_enable(ARRAY_SIZE(modi->supplies), modi->supplies);
	// if (ret < 0) {
	// 	dev_err(modi->dev, "unable to enable regulators\n");
	// 	return ret;
	// }
	//msleep(20); // 等待20毫秒

	/* 设置RESET引脚为低电平, 复位 ... */
	//gpiod_set_value(modi->reset_gpio, 0);

	//msleep(20); // 等待10毫秒

	/* 设置RESET引脚为高电平, 退出复位. */
	//gpiod_set_value(modi->reset_gpio, 1);
	//msleep(20); // 等待10毫秒
	//printk("WL---> modi rgb panel reset-gpio value: %d\n", gpiod_get_value(modi->reset_gpio));
	return 0;
}

/**
 * 面板电源关闭函数，关闭面板电源。
 * @modi: 指向modipanel结构的指针。
 * 返回值：成功返回0，失败返回错误码。
 */
static int modi_power_off(struct modipanel *modi)
{
	//return regulator_bulk_disable(ARRAY_SIZE(modi->supplies), modi->supplies);
	printk("WL---> modi power off, %d.\n", __LINE__);
	return 0;
}

/**
 * 禁用面板函数，在禁用面板时调用。
 * @panel: DRM面板指针。
 * 返回值：总是返回0。
 */
static int modi_disable(struct drm_panel *panel)
{
	printk("WL---> modi disable, %d.\n", __LINE__);
	return 0;
}

/**
 * 取消准备面板函数，在取消准备面板时调用。
 * @panel: DRM面板指针。
 * 返回值：成功返回0，失败返回错误码。
 */
static int modi_unprepare(struct drm_panel *panel)
{
	struct modipanel *modi = panel_to_modi(panel);
	printk("WL---> modi unprepare, %d.\n", __LINE__);
	return modi_power_off(modi);
}

/**
 * 准备面板函数，在准备面板时调用。
 * @panel: DRM面板指针。
 * 返回值：成功返回0，失败返回错误码。
 */
static int modi_prepare(struct drm_panel *panel)
{
	struct modipanel *modi = panel_to_modi(panel);
	int ret;

	ret = modi_power_on(modi);
	if (ret < 0)
		return ret;

	ret = modi_init(panel, modi); // 初始化面板（假设存在一个modi_init函数）
	if (ret < 0)
		modi_unprepare(panel);

	printk("WL---> modi prepare, %d.\n", __LINE__);
	return ret;
}

/**
 * 启用面板函数，在启用面板时调用。
 * @panel: DRM面板指针。
 * 返回值：总是返回0。
 */
static int modi_enable(struct drm_panel *panel)
{
	printk("WL---> modi enable, %d.\n", __LINE__);
	return 0;
}

/**
 * 定义了一个480x800分辨率的显示模式。
 */
static const struct drm_display_mode prgb_480x800_mode = {
 	.clock = 30000, // 像素时钟频率，单位kHz。
 	.hdisplay = 480, // 水平显示宽度（HDP + 1），即可视区域的像素数
 	.hsync_start = 488, // 水平同步信号开始位置（LPS + HDP），即从行开始到HSYNC开始的像素数
 	.hsync_end = 492, // 水平同步信号结束位置（hsync_start + HPW），即从行开始到HSYNC结束的像素数
 	.htotal = 580, // 总水平周期（HT + 1），即一行总的像素数
 	.vdisplay = 800, // 垂直显示高度（VDP + 1），即可视区域的行数
 	.vsync_start = 810, // 垂直同步信号开始位置（FPS + VDP），即从帧开始到VSYNC开始的行数
 	.vsync_end = 812, // 垂直同步信号结束位置（vsync_start + VPW），即从帧开始到VSYNC结束的行数
 	.vtotal = 830, // 总垂直周期（VT + 1），即一帧总的行数
 	.flags = 0,//DRM_MODE_FLAG_PHSYNC|DRM_MODE_FLAG_PVSYNC // 标志位，可根据需要设置其他标志
};

//  static const struct drm_display_mode prgb_480x800_mode = {
//  	.clock = 30000, // 像素时钟频率，单位kHz。
//  	.hdisplay = 480, // 水平显示宽度（HDP + 1），即可视区域的像素数
//  	.hsync_start = 482, // 水平同步信号开始位置（LPS + HDP），即从行开始到HSYNC开始的像素数
//  	.hsync_end = 484, // 水平同步信号结束位置（hsync_start + HPW），即从行开始到HSYNC结束的像素数
//  	.htotal = 486, // 总水平周期（HT + 1），即一行总的像素数
//  	.vdisplay = 800, // 垂直显示高度（VDP + 1），即可视区域的行数
//  	.vsync_start = 802, // 垂直同步信号开始位置（FPS + VDP），即从帧开始到VSYNC开始的行数
//  	.vsync_end = 804, // 垂直同步信号结束位置（vsync_start + VPW），即从帧开始到VSYNC结束的行数
//  	.vtotal = 806, // 总垂直周期（VT + 1），即一帧总的行数
//  	.flags = 0,//DRM_MODE_FLAG_PHSYNC|DRM_MODE_FLAG_PVSYNC // 标志位，可根据需要设置其他标志
//  };

/**
 * 获取面板支持的显示模式。
 * @panel: DRM面板指针。
 * @connector: DRM连接器指针。
 * 返回值：成功返回模式数量，失败返回错误码。
 */
static int modi_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	//struct modipanel *modi = panel_to_modi(panel);
	struct drm_device *drm = connector->dev;
	struct drm_display_mode *mode;
	//struct drm_display_info *info;

	//info = &connector->display_info;
	//info->width_mm = modi->conf->width_mm; // 设置物理宽度
	//info->height_mm = modi->conf->height_mm; // 设置物理高度

	// 根据配置设置像素数据驱动边沿
	//if (modi->conf->dclk_active_high)
	//info->bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE;
	//else
	//	info->bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

	// 设置DE信号极性
	//if (modi->conf->de_active_high)
	//info->bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	//else
	//	info->bus_flags |= DRM_BUS_FLAG_DE_LOW;

	// 根据输入模式选择显示模式
	// switch (modi->input) {
	// case MODI_INPUT_PRGB_THROUGH:
	// case MODI_INPUT_PRGB_ALIGNED:
	// 	mode = drm_mode_duplicate(drm, &prgb_480x800_mode);
	// 	break;
	// default:
	// 	mode = NULL;
	// 	break;
	// }

	mode = drm_mode_duplicate(drm, &prgb_480x800_mode);
	if (!mode) {
		dev_err(panel->dev, "bad mode or failed to add mode.\n");
		printk("WL---> bad mode or failed to add mode.\n");
		return -EINVAL;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	// // 设置同步信号极性
	// if (modi->conf->hsync_active_high)
	//mode->flags |= DRM_MODE_FLAG_PHSYNC;
	// else
	// 	mode->flags |= DRM_MODE_FLAG_NHSYNC;
	// if (modi->conf->vsync_active_high)
	// 	mode->flags |= DRM_MODE_FLAG_PVSYNC;
	// else
	// 	mode->flags |= DRM_MODE_FLAG_NVSYNC;

	// mode->width_mm = modi->conf->width_mm;
	// mode->height_mm = modi->conf->height_mm;
	//drm_mode_probed_add(connector, mode);info

	printk("WL---> display mode:%dx%d\n",mode->hdisplay, mode->vdisplay);
	printk("WL---> timing: clock=%dHz, htotal=%d, hsync=%d, vtotal=%d, vsync=%d\n", 
		mode->clock, mode->htotal, mode->hsync_start, mode->vtotal, mode->vsync_start);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	printk("WL---> drm mode(%s) duplicate ok. %d.\n", mode->name, __LINE__);
	return 1; /* 返回模式数量 */
}

/**
 * 定义DRM面板功能集合。
 */
static const struct drm_panel_funcs modipanel_drm_funcs = {
	.disable = modi_disable, // 禁用面板
	.unprepare = modi_unprepare, // 取消准备面板
	.prepare = modi_prepare, // 准备面板
	.enable = modi_enable, // 启用面板
	.get_modes = modi_get_modes, // 获取支持的显示模式
};

/**
 * 探测函数，当设备被发现时调用。
 * @spi: SPI设备指针。
 * 返回值：成功返回0，失败返回错误码。
 */
static int modi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev; // 获取设备结构体指针
	struct modipanel *modi;
	const struct regmap_config *regmap_config;
	// u8 gamma;
	// u32 val;
	int ret;
	// int i;

	printk("WL---> modi rgb panel probe ...\n");

	// 动态分配内存用于存储modipanel结构体
	modi = devm_kzalloc(dev, sizeof(struct modipanel), GFP_KERNEL);
	if (!modi)
		return -ENOMEM;

	// 设置SPI设备的私有数据为modi
	spi_set_drvdata(spi, modi);

	modi->dev = dev;

	// 从设备树中获取匹配的数据（配置信息）
	// modi->conf = of_device_get_match_data(dev);
	// if (!modi->conf) {
	// 	dev_err(dev, "missing device configuration\n");
	// 	printk("WL---> missing device configuration.\n");
	// 	return -ENODEV;
	// }	

    // 处理VREG1OUT电压设置
//     val = modi->conf->vreg1out_mv;
//     if (!val) {
//         /* 如果未指定，则使用最大值 */
//         modi->vreg1out = U8_MAX;
//     } else {
//         if (val < 3600 || val > 6000 || (val % 100) != 0) {
//             dev_err(dev, "invalid VREG1OUT value\n");
//             return -EINVAL;
//         }
//         val -= 3600;
//         val /= 100;
//         modi->vreg1out = val;
//     }

    // 处理VCOM HIGH百分比设置
//     val = modi->conf->vcom_high_percent;
//     if (!val) {
//         modi->vcom_high = U8_MAX;
//     } else {
//         if (val < 37 || val > 100) {
//             dev_err(dev, "invalid VCOM high percentage\n");
//             return -EINVAL;
//         }
//         val -= 37;
//         modi->vcom_high = val;
//     }

    // 处理VCOM幅度百分比设置
//     val = modi->conf->vcom_amplitude_percent;
//     if (!val) {
//         modi->vcom_amplitude = U8_MAX;
//     } else {
//         if (val < 70 || val > 132) {
//             dev_err(dev, "invalid VCOM amplitude percentage\n");
//             return -EINVAL;
//         }
//         val -= 70;
//         val >>= 1; // 转换为每2%
//         modi->vcom_amplitude = val;
//     }

    // 初始化伽马校正系数
//     for (i = 0; i < ARRAY_SIZE(modi->gamma); i++) {
//         val = modi->conf->gamma_corr_neg[i];
//         if (val > 15) val = 15;
//         gamma = val << 4;
//         val = modi->conf->gamma_corr_pos[i];
//         if (val > 15) val = 15;
//         gamma |= val;
//         modi->gamma[i] = gamma;
//     }

    // 初始化电源供应器
//     modi->supplies[0].supply = "vcc"; 
//     modi->supplies[1].supply = "iovcc"; 
//     modi->supplies[2].supply = "vci"; 
//     ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(modi->supplies), modi->supplies);
//     if (ret < 0)
//         return ret;
    
//     // 设置电源供应器电压
//     ret = regulator_set_voltage(modi->supplies[0].consumer, 2700000, 3600000);
//     if (ret) return ret;
//     ret = regulator_set_voltage(modi->supplies[1].consumer, 1650000, 3600000);
//     if (ret) return ret;
//     ret = regulator_set_voltage(modi->supplies[2].consumer, 2700000, 3600000);
//     if (ret) return ret;

    // 获取RESET GPIO
    // modi->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    // if (IS_ERR(modi->reset_gpio)) {
	// 	dev_err(dev, "failed to get RESET GPIO\n");
	// 	return PTR_ERR(modi->reset_gpio);
	// }

	// printk("WL---> modi panel got reset-gpio.\n");
	// printk("WL---> default reset-gpio value: %d\n", gpiod_get_value(modi->reset_gpio));
	// // /* 设置RESET引脚为高电平, 退出复位. */
	// gpiod_set_value(modi->reset_gpio, 1);
	// msleep(20); // 等待10毫秒
	// printk("WL---> set hight and read, reset-gpio value: %d\n", gpiod_get_value(modi->reset_gpio));

    // modi->cs_gpio = devm_gpiod_get_optional(dev, "cs", GPIOD_OUT_HIGH);
    // if (IS_ERR(modi->cs_gpio)) {
	// 	dev_err(dev, "failed to get CS GPIO\n");
	// 	return PTR_ERR(modi->cs_gpio);
	// }

	// printk("WL---> modi panel got cs-gpio.\n");
	// printk("WL---> default cs-gpio value: %d\n", gpiod_get_value(modi->cs_gpio));
	// gpiod_set_value(modi->cs_gpio, 0); // selected.
	// msleep(20); // 等待10毫秒
	// printk("WL---> set cs low and read, cs-gpio value: %d\n", gpiod_get_value(modi->cs_gpio));	

    // 配置SPI参数
    spi->bits_per_word = 9;
	spi->mode = 0; //CPOL=0, CPHA=0
    ret = spi_setup(spi);
    if (ret < 0) {
        dev_err(dev, "spi setup failed.\n");
        return ret;
    }

	printk("WL---> spi mode: %d\n", spi->mode);
	printk("WL---> spi max speed hz: %d\n", spi->max_speed_hz);
	printk("WL---> spi bits per word: %d\n", spi->bits_per_word);

	// 初始化寄存器映射
	regmap_config = &modipanel_regmap_config;
	modi->regmap = devm_regmap_init(dev, &modipanel_regmap_bus, dev, regmap_config);
	if (IS_ERR(modi->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		printk("WL---> failed to allocate register map.\n");
		return PTR_ERR(modi->regmap);
	}
	
	// 根据配置确定输入模式
	// if (modi->conf->input == MODI_INPUT_UNKNOWN) {
	// 	ret = regmap_read(modi->regmap, ENTRY, &val);
	// 	if (ret) {
	// 		dev_err(dev, "can't get entry setting (%d)\n", ret);
	// 		return ret;
	// 	}

	// 	modi->input = (val >> 4) & 0x0f;
	// 	if (modi->input >= MODI_INPUT_UNKNOWN)
	// 		modi->input = MODI_INPUT_UNKNOWN;
	// } else {
	// 	modi->input = modi->conf->input;
	// }

	//modi输入模式为并行RGB24,直通模式.
	modi->input = MODI_INPUT_PRGB_THROUGH;

	// 初始化DRM面板
	drm_panel_init(&modi->panel, dev, &modipanel_drm_funcs, DRM_MODE_CONNECTOR_DPI);
	drm_panel_add(&modi->panel);
	printk("WL---> modi rgb panel done.\n");
	return 0;
}

/**
 * 移除函数，在设备移除时调用。
 * @spi: SPI设备指针。
 */
static void modi_remove(struct spi_device *spi)
{
    struct modipanel *modi = spi_get_drvdata(spi);

    modi_power_off(modi);
    drm_panel_remove(&modi->panel);
	printk("WL---> modi remove, %d.\n", __LINE__);
}

// 设备树匹配表
static const struct of_device_id modi_of_match[] = {
	{ .compatible = "modi,modi-rgb-panel", .data = NULL },
    	{ }
};
MODULE_DEVICE_TABLE(of, modi_of_match);

// SPI设备ID表
static const struct spi_device_id modi_id[] = {
	{ "modi-rgb-panel", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, modi_id);

// SPI驱动程序结构体
static struct spi_driver modi_driver = {
	.probe = modi_probe,
	.remove = modi_remove,
	.id_table = modi_id,
	.driver = {
		.name = "modi_rgb_panel",
		.of_match_table = modi_of_match,
    	},
};
module_spi_driver(modi_driver);

MODULE_AUTHOR("Jack WANG <553888542@qq.com>");
MODULE_DESCRIPTION("MODI LCD panel driver");
MODULE_LICENSE("GPL v2");
