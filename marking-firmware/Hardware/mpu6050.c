#include "stm32f10x.h"
#include "mpu6050.h"

/* ==========================================================================
 * 分三段：引脚层 / 协议层 / 器件层。
 *
 * 课堂例程把前两段拆成了 MyI2C.c + MPU6050.c 两个文件。这里合并成一个：
 * 全项目只有 MPU6050 一个 I2C 从机（OLED 有它自己的位操作实现，不走这套），
 * 拆开只会多两个工程节点、多一次跨文件找引脚。段与段之间用注释隔开，
 * 将来真要接第二个器件，把前两段原样剪出去就是 MyI2C.c。
 * ========================================================================== */

/* ---------- 寄存器 ---------- */
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C
#define MPU_REG_WHO_AM_I        0x75

#define MPU_ADDR                0xD0    /* 7 位地址 0x68 左移一位，写方向 */
#define MPU_ADDR_READ           0xD1
#define MPU_ID_EXPECTED         0x68

/* 量程 ±500 dps。
 *
 * 课堂例程用 ±2000（16.4 LSB/dps），那是"甩着玩"的配置。这台车画线时
 * 的偏航角速度是几度每秒的量级，±500（65.5 LSB/dps）分辨率高 4 倍，
 * 静止时的量化噪声直接小 4 倍 —— 积分的漂移主要就来自这部分噪声。
 * 原地转 90 度用时 2 秒的话约 45dps，离 500 还有十倍余量。 */
#define MPU_GYRO_FS_SEL         0x08    /* FS_SEL = 1 -> ±500 dps */
#define MPU_GYRO_LSB_PER_DPS    65.5f

/* DLPF_CFG = 3：44Hz 带宽，延迟 4.9ms。
 *
 * 带宽要和采样率配：我们 100Hz 采样（10ms 一次），44Hz 带宽刚好在奈奎斯特
 * 频率附近，既能滤掉电机振动的高频分量，又不会让转向的相位滞后太多。
 * 用 260Hz（DLPF 关闭）会把步进电机的振动全收进来，积分两天就飘没了。 */
#define MPU_DLPF_CFG            0x03    /* CONFIG 低 3 位 */
#define MPU_SMPLRT_DIV_VAL      9       /* 1kHz / (1+9) = 100Hz */

/* ---------- 引脚层 ---------- */

/* 半周期延时。volatile 循环在 72MHz 下每次约 5 个周期，
 * 40 次 ≈ 200 周期 ≈ 2.8us，对应约 180kHz 的 SCL。
 * 不去追求精确的 100kHz：软件 I2C 只要落在 10k~400k 之间都能通，
 * 而 I2C_DELAY_LOOPS 调大调小是现场唯一需要动的地方（线长、上拉弱就调大）。 */
#define I2C_DELAY_LOOPS     40

static void i2c_delay(void)
{
    volatile uint8_t i = I2C_DELAY_LOOPS;
    while (i--) {
    }
}

#define SCL_H()     GPIO_SetBits(MPU_SCL_PORT, MPU_SCL_PIN)
#define SCL_L()     GPIO_ResetBits(MPU_SCL_PORT, MPU_SCL_PIN)
#define SDA_H()     GPIO_SetBits(MPU_SDA_PORT, MPU_SDA_PIN)
#define SDA_L()     GPIO_ResetBits(MPU_SDA_PORT, MPU_SDA_PIN)
#define SDA_READ()  GPIO_ReadInputDataBit(MPU_SDA_PORT, MPU_SDA_PIN)

/* 粗延时，只用于等 MPU6050 上电稳定，不参与任何时序。
 * 这时候主循环还没跑起来，用不了 timer_counter。
 * 12000 次 × 约 6 周期 = 72000 周期 ≈ 1ms @72MHz。
 * 只求量级对，不求准 —— 手册要求 100ms，给到 80ms 还是 120ms 都无所谓，
 * 因为后面还有回读校验兜底，真不够会返回错误码而不是给出假数据。 */
static void delay_ms_coarse(uint16_t ms)
{
    while (ms--) {
        volatile uint16_t i = 12000;
        while (i--) {
        }
    }
}

static void i2c_pins_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 开漏输出。开漏是 I2C 的电气前提：总线上所有设备只能"拉低"，
     * 不能"推高"，高电平由上拉电阻给。用推挽的话，主机输出高、
     * 从机输出低，就是两个 MOS 管直接对怼。 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin   = MPU_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU_SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = MPU_SDA_PIN;
    GPIO_Init(MPU_SDA_PORT, &GPIO_InitStructure);

    /* 开漏模式下写 1 = 释放总线（让上拉拉高） */
    SCL_H();
    SDA_H();
}

/* 总线恢复：SDA 被某个从机一直拉着不放时，主机发 STOP 也放不开，
 * 之后就永远读不到数据，看起来像"传感器坏了"。
 *
 * 唯一的出路是补 9 个时钟：从机手里那个字节（含应答位）最多 9 位，
 * 给够 9 个 SCL，它会自己数完并把 SDA 交出来。最后补一个 STOP，
 * 让所有设备都回到空闲态。
 *
 * 上电时做一次，读失败时也做一次 —— 这一步能救回绝大多数"插拔过传感器
 * 之后就不认了"的现场故障，比重上电快得多。 */
static void i2c_bus_recover(void)
{
    uint8_t i;

    SDA_H();
    for (i = 0; i < 9; i++) {
        SCL_L();
        i2c_delay();
        SCL_H();
        i2c_delay();
    }

    /* STOP：SCL 高电平期间，SDA 由低变高 */
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_H();
    i2c_delay();
}

/* ---------- 协议层 ---------- */

static void i2c_start(void)
{
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_L();        /* SCL 高电平期间拉低 SDA = 起始 */
    i2c_delay();
    SCL_L();
    i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L();
    i2c_delay();
    SCL_H();
    i2c_delay();
    SDA_H();        /* SCL 高电平期间释放 SDA = 终止 */
    i2c_delay();
}

/* 返回 0 = 从机应答，1 = 无应答 */
static uint8_t i2c_send_byte(uint8_t byte)
{
    uint8_t i;
    uint8_t nak;

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        byte <<= 1;
        i2c_delay();
        SCL_H();
        i2c_delay();
        SCL_L();
    }

    /* 第 9 个时钟：主机释放 SDA，读从机拉不拉低。
     * 课堂例程在这里调 MyI2C_ReceiveAck() 但把返回值丢掉了 ——
     * 传感器没插上时，后面每一句都会"成功"执行，最后拿到一串 0xFF，
     * 换算出来就是 255 dps 的角速度。这个返回值必须传上去。 */
    SDA_H();
    i2c_delay();
    SCL_H();
    i2c_delay();
    nak = (uint8_t)(SDA_READ() ? 1 : 0);
    SCL_L();
    i2c_delay();

    return nak;
}

static uint8_t i2c_recv_byte(void)
{
    uint8_t i;
    uint8_t byte = 0;

    SDA_H();        /* 释放 SDA，交给从机驱动 */
    for (i = 0; i < 8; i++) {
        i2c_delay();
        SCL_H();
        i2c_delay();
        byte <<= 1;
        if (SDA_READ()) {
            byte |= 0x01;
        }
        SCL_L();
    }

    return byte;
}

/* ack = 0 应答（还要继续读），1 非应答（这是最后一个字节） */
static void i2c_send_ack(uint8_t ack)
{
    if (ack) {
        SDA_H();
    } else {
        SDA_L();
    }
    i2c_delay();
    SCL_H();
    i2c_delay();
    SCL_L();
    i2c_delay();
    SDA_H();
}

/* ---------- 器件层 ---------- */

static uint8_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t err;

    i2c_start();
    err  = i2c_send_byte(MPU_ADDR);
    err |= i2c_send_byte(reg);
    err |= i2c_send_byte(value);
    i2c_stop();

    return err ? MPU_ERR_I2C : MPU_OK;
}

/* 连续读 len 个寄存器。一次 START + 重复起始，中途不停总线 ——
 * 这样读到的多个字节保证是同一采样时刻的快照（见 mpu6050.h 里的说明）。 */
static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) {
        return MPU_OK;
    }

    i2c_start();
    if (i2c_send_byte(MPU_ADDR)) {
        i2c_stop();
        return MPU_ERR_I2C;
    }
    if (i2c_send_byte(reg)) {
        i2c_stop();
        return MPU_ERR_I2C;
    }

    i2c_start();                        /* 重复起始：方向从写翻成读 */
    if (i2c_send_byte(MPU_ADDR_READ)) {
        i2c_stop();
        return MPU_ERR_I2C;
    }

    while (len--) {
        *buf++ = i2c_recv_byte();
        i2c_send_ack(len ? 0 : 1);      /* 最后一个字节必须回 NAK */
    }

    i2c_stop();
    return MPU_OK;
}

/* ---------- 对外 ---------- */

static uint8_t  mpu_online = 0;
static uint16_t mpu_err_count = 0;

/* 写全部配置寄存器。Init 和 Recover 共用这一份 ——
 * 两处各写一遍的话，改量程时只改一处，另一条路径就会带着旧配置跑，
 * 而且不会有任何报错（角度直接差一个倍数）。 */
static uint8_t mpu_apply_config(void)
{
    /* 唤醒 + 时钟源选 X 轴陀螺 PLL。
     * 不选内部 8MHz RC：那个振荡器温漂大，直接变成零偏漂移。 */
    if (mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (mpu_write_reg(MPU_REG_PWR_MGMT_2, 0x00) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (mpu_write_reg(MPU_REG_SMPLRT_DIV, MPU_SMPLRT_DIV_VAL) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (mpu_write_reg(MPU_REG_CONFIG, MPU_DLPF_CFG) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (mpu_write_reg(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_SEL) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    /* 加速度计也开一下：现在不用，但姿态解算（倾角补偿航向）以后要用，
     * 而且它工作时芯片的供电更稳定。±4g，够。 */
    if (mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x08) != MPU_OK) {
        return MPU_ERR_I2C;
    }

    return MPU_OK;
}

/* 回读校验。
 * 只相信"写进去还能读出来"的配置：I2C 通但不稳定的模块（供电不足、
 * 上拉太弱）会出现写成功、实际没生效，然后所有读数都是 0。
 * 那比完全没有陀螺仪更糟 —— 闭环会认为车一直走得很直。 */
static uint8_t mpu_verify_config(void)
{
    uint8_t back = 0;

    if (mpu_read_regs(MPU_REG_GYRO_CONFIG, &back, 1) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (back != MPU_GYRO_FS_SEL) {
        return MPU_ERR_CONFIG;
    }

    return MPU_OK;
}

uint8_t MPU6050_WhoAmI(uint8_t *id)
{
    return mpu_read_regs(MPU_REG_WHO_AM_I, id, 1);
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;
    uint8_t ret;

    mpu_online = 0;

    i2c_pins_init();
    i2c_bus_recover();

    /* MPU6050 上电到能应答 I2C 大约 100ms（手册 tPOWERUP）。
     * 等不够的话 WHO_AM_I 会读到 0xFF，看起来像"芯片不在"。 */
    delay_ms_coarse(120);

    if (mpu_read_regs(MPU_REG_WHO_AM_I, &id, 1) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (id != MPU_ID_EXPECTED) {
        return MPU_ERR_ID;
    }

    /* 软复位：不管上电前它是什么状态（可能被上一次运行留在睡眠、
     * 或者量程是 ±2000），先按手册写 0x80 复位，等它自己起来。
     * 不复位的话，配置寄存器里那些我们没写的位会保持上一次的值。 */
    if (mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    delay_ms_coarse(100);

    ret = mpu_apply_config();
    if (ret != MPU_OK) {
        return ret;
    }

    delay_ms_coarse(10);

    ret = mpu_verify_config();
    if (ret != MPU_OK) {
        return ret;
    }

    mpu_online = 1;
    mpu_err_count = 0;

    return MPU_OK;
}

uint8_t MPU6050_Recover(void)
{
    uint8_t id = 0;
    uint8_t ret;

    mpu_online = 0;

    /* 引脚可能被别处的 GPIO 初始化改过（比如某次 OLED 或串口初始化
     * 顺手重配了整个 GPIOB），重设一遍最省事。 */
    i2c_pins_init();
    i2c_bus_recover();

    if (mpu_read_regs(MPU_REG_WHO_AM_I, &id, 1) != MPU_OK) {
        return MPU_ERR_I2C;
    }
    if (id != MPU_ID_EXPECTED) {
        return MPU_ERR_ID;
    }

    /* 不做软复位、不延时：运行中调用，卡太久会撑爆串口环形缓冲 */
    ret = mpu_apply_config();
    if (ret != MPU_OK) {
        return ret;
    }

    ret = mpu_verify_config();
    if (ret != MPU_OK) {
        return ret;
    }

    mpu_online = 1;

    return MPU_OK;
}

/* 把 6 个原始字节拼成三个 int16，大端。单独抽出来是因为读失败的时候
 * 一个字节都不该往外写 —— 让调用方拿到上一次的值，比拿到半新半旧的
 * 拼接值安全（半新半旧会产生一个根本不存在的角速度尖峰）。 */
uint8_t MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    uint8_t ret;

    if (!mpu_online) {
        return MPU_ERR_NOT_INIT;
    }

    ret = mpu_read_regs(MPU_REG_GYRO_XOUT_H, buf, 6);
    if (ret != MPU_OK) {
        mpu_err_count++;
        /* 掉线了就试一次总线恢复。成功与否都留给下一次调用去验证，
         * 这里不重试读 —— 读半截的时候重试会把时序拉得很长，
         * 而调用方（10ms 轮询）本来就会再来。 */
        i2c_bus_recover();
        mpu_online = 0;
        return ret;
    }

    *gx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *gy = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *gz = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    return MPU_OK;
}

uint8_t MPU6050_ReadGyroZ(int16_t *gz)
{
    int16_t gx, gy;

    return MPU6050_ReadGyro(&gx, &gy, gz);
}

uint8_t MPU6050_IsOnline(void)
{
    return mpu_online;
}

uint16_t MPU6050_ErrorCount(void)
{
    return mpu_err_count;
}
