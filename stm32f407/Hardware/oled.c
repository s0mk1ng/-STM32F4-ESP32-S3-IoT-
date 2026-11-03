#include "OLED.h"
#include "i2c.h"
#include <string.h>

/* OLED使用的I2C地址 */
static uint16_t oled_i2c_addr = OLED_I2C_ADDR_3C;

/* 128x64 / 8 = 1024 Bytes */
static uint8_t OLED_GRAM[OLED_WIDTH * OLED_HEIGHT / 8U];

/*-----------------------------------------------------------
 * 向SSD1306发送命令
 *----------------------------------------------------------*/
static HAL_StatusTypeDef OLED_WriteCommand(uint8_t cmd)
{
    uint8_t data[2];

    data[0] = 0x00U;     /* Co=0, D/C#=0：后面是命令 */
    data[1] = cmd;

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        oled_i2c_addr,
        data,
        2U,
        100U
    );
}

/*-----------------------------------------------------------
 * 向SSD1306发送一组命令
 *----------------------------------------------------------*/
static HAL_StatusTypeDef OLED_WriteCommands(const uint8_t *cmds,uint16_t len)
{
    uint8_t buffer[32];

    if ((cmds == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    while (len > 0U)
    {
        uint16_t send_len = (len > 31U) ? 31U : len;

        buffer[0] = 0x00U;

        memcpy(&buffer[1], cmds, send_len);

        if (HAL_I2C_Master_Transmit(
                &hi2c1,
                oled_i2c_addr,
                buffer,
                send_len + 1U,
                100U) != HAL_OK)
        {
            return HAL_ERROR;
        }

        cmds += send_len;
        len  -= send_len;
    }

    return HAL_OK;
}

/*-----------------------------------------------------------
 * OLED初始化
 *----------------------------------------------------------*/
void OLED_Init(void)
{
    const uint8_t init_cmds[] =
    {
        0xAE,       /* Display OFF */

        0xD5, 0x80, /* Set Display Clock Divide Ratio/Oscillator */

        0xA8, 0x3F, /* Multiplex Ratio = 63 */

        0xD3, 0x00, /* Display Offset = 0 */

        0x40,       /* Display Start Line = 0 */

        0x8D, 0x14, /* Charge Pump Enable */

        0x20, 0x02, /* Memory Addressing Mode = Page */

        0xA1,       /* Segment Remap */

        0xC8,       /* COM Output Scan Direction */

        0xDA, 0x12, /* COM Pins Hardware Configuration */

        0x81, 0xCF, /* Contrast */

        0xD9, 0xF1, /* Pre-charge Period */

        0xDB, 0x40, /* VCOMH Deselect Level */

        0xA4,       /* Entire Display ON from RAM */

        0xA6,       /* Normal Display */

        0xAF        /* Display ON */
    };

    HAL_Delay(100U);

    /*-------------------------------------------------------
     * 尝试0x3C
     *------------------------------------------------------*/
    if (HAL_I2C_IsDeviceReady(
            &hi2c1,
            OLED_I2C_ADDR_3C,
            3U,
            100U) == HAL_OK)
    {
        oled_i2c_addr = OLED_I2C_ADDR_3C;
    }
    /*-------------------------------------------------------
     * 如果0x3C不行，再尝试0x3D
     *------------------------------------------------------*/
    else if (HAL_I2C_IsDeviceReady(
                 &hi2c1,
                 OLED_I2C_ADDR_3D,
                 3U,
                 100U) == HAL_OK)
    {
        oled_i2c_addr = OLED_I2C_ADDR_3D;
    }
    else
    {
        /* OLED没有响应 */
        return;
    }

    OLED_WriteCommands(init_cmds,
                       (uint16_t)sizeof(init_cmds));

    OLED_Clear();
    OLED_Update();
}

/*-----------------------------------------------------------
 * 清空显存
 *----------------------------------------------------------*/
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
}

/*-----------------------------------------------------------
 * 整屏填充
 * value = 0x00 -> 全黑
 * value = 0xFF -> 全亮
 *----------------------------------------------------------*/
void OLED_Fill(uint8_t value)
{
    memset(OLED_GRAM, value, sizeof(OLED_GRAM));
}

/*-----------------------------------------------------------
 * 刷新OLED
 *
 * OLED_GRAM：
 * 1024 Bytes
 *
 * SSD1306：
 * 8个Page
 * 每个Page 128 Bytes
 *----------------------------------------------------------*/
void OLED_Update(void)
{
    uint8_t buffer[17];
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++)
    {
        /* 设置Page地址 */
        OLED_WriteCommand((uint8_t)(0xB0U + page));

        /* 设置列地址低4位 */
        OLED_WriteCommand(0x00U);

        /* 设置列地址高4位 */
        OLED_WriteCommand(0x10U);

        /*---------------------------------------------------
         * 一次发送16个像素数据
         * 第0字节：0x40 -> 后面为显示数据
         *--------------------------------------------------*/
        for (column = 0U; column < 128U; column += 16U)
        {
            buffer[0] = 0x40U;

            memcpy(
                &buffer[1],
                &OLED_GRAM[page * 128U + column],
                16U
            );

            HAL_I2C_Master_Transmit(
                &hi2c1,
                oled_i2c_addr,
                buffer,
                17U,
                100U
            );
        }
    }
}

/*-----------------------------------------------------------
 * OLED分块刷新
 *
 * page   : 0~7
 * column : 0~120，每次增加8
 *
 * 每次只发送8字节显示数据
 * 不在本函数内部获取Mutex，由调用任务负责互斥保护
 *----------------------------------------------------------*/
HAL_StatusTypeDef OLED_UpdateChunk(uint8_t page, uint8_t column)
{
    uint8_t addr_cmds[3];
    uint8_t data[OLED_UPDATE_CHUNK_SIZE + 1U];
    uint16_t offset;

    /* 参数检查 */
    if ((page >= (OLED_HEIGHT / 8U)) ||
        (column >= OLED_WIDTH) ||
        ((column % OLED_UPDATE_CHUNK_SIZE) != 0U))
    {
        return HAL_ERROR;
    }

    /* 设置Page和Column地址 */
    addr_cmds[0] = (uint8_t)(0xB0U | page);

    addr_cmds[1] = (uint8_t)(column & 0x0FU);

    addr_cmds[2] = (uint8_t)(0x10U | ((column >> 4U) & 0x0FU));

    /* 发送地址设置命令 */
    if (OLED_WriteCommands(addr_cmds, sizeof(addr_cmds)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 第一个字节0x40表示后续为显示数据 */
    data[0] = 0x40U;

    /* 计算显存偏移 */
    offset = (uint16_t)page * OLED_WIDTH + column;

    /* 复制8字节显存数据 */
    memcpy(
        &data[1],
        &OLED_GRAM[offset],
        OLED_UPDATE_CHUNK_SIZE
    );

    /* 发送显示数据 */
    return HAL_I2C_Master_Transmit(
        &hi2c1,
        oled_i2c_addr,
        data,
        sizeof(data),
        100U
    );
}

/*-----------------------------------------------------------
 * OLED测试
 *
 * 全亮 -> 延时 -> 全灭
 *----------------------------------------------------------*/
void OLED_Test(void)
{
    OLED_Fill(0xFFU);
    OLED_Update();

    HAL_Delay(1000U);

    OLED_Fill(0x00U);
    OLED_Update();

    HAL_Delay(1000U);

    OLED_Fill(0xFFU);
    OLED_Update();
}

/*-----------------------------------------------------------
 * 获取5x7字符字模
 *
 * 每个字符使用5个字节：
 * 每个字节表示一列
 * 每个字节只有低7位有效
 *----------------------------------------------------------*/
static void OLED_GetGlyph(char c, uint8_t *glyph)
{
    memset(glyph, 0x00, 5U);

    switch (c)
    {
        case 'A':
            glyph[0] = 0x7E;
            glyph[1] = 0x11;
            glyph[2] = 0x11;
            glyph[3] = 0x7E;
            break;

        case 'B':
            glyph[0] = 0x7F;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x36;
            break;

        case 'C':
            glyph[0] = 0x3E;
            glyph[1] = 0x41;
            glyph[2] = 0x41;
            glyph[3] = 0x22;
            break;

        case 'D':
            glyph[0] = 0x7F;
            glyph[1] = 0x41;
            glyph[2] = 0x41;
            glyph[3] = 0x3E;
            break;

        case 'E':
            glyph[0] = 0x7F;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x41;
            break;

        case 'G':
            glyph[0] = 0x3E;
            glyph[1] = 0x41;
            glyph[2] = 0x49;
            glyph[3] = 0x7A;
            break;

        case 'H':
            glyph[0] = 0x7F;
            glyph[1] = 0x08;
            glyph[2] = 0x08;
            glyph[3] = 0x7F;
            break;

        case 'I':
            glyph[0] = 0x41;
            glyph[1] = 0x7F;
            glyph[2] = 0x41;
            break;

        case 'K':
            glyph[0] = 0x7F;
            glyph[1] = 0x08;
            glyph[2] = 0x14;
            glyph[3] = 0x22;
            break;

        case 'L':
            glyph[0] = 0x7F;
            glyph[1] = 0x40;
            glyph[2] = 0x40;
            glyph[3] = 0x40;
            break;

        case 'M':
            glyph[0] = 0x7F;
            glyph[1] = 0x02;
            glyph[2] = 0x0C;
            glyph[3] = 0x02;
            glyph[4] = 0x7F;
            break;

        case 'N':
            glyph[0] = 0x7F;
            glyph[1] = 0x04;
            glyph[2] = 0x08;
            glyph[3] = 0x10;
            glyph[4] = 0x7F;
            break;

        case 'O':
            glyph[0] = 0x3E;
            glyph[1] = 0x41;
            glyph[2] = 0x41;
            glyph[3] = 0x3E;
            break;

        case 'P':
            glyph[0] = 0x7F;
            glyph[1] = 0x09;
            glyph[2] = 0x09;
            glyph[3] = 0x06;
            break;

        case 'R':
            glyph[0] = 0x7F;
            glyph[1] = 0x09;
            glyph[2] = 0x19;
            glyph[3] = 0x26;
            break;

        case 'S':
            glyph[0] = 0x26;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x32;
            break;

        case 'T':
            glyph[0] = 0x01;
            glyph[1] = 0x7F;
            glyph[2] = 0x01;
            glyph[3] = 0x01;
            break;

        case 'U':
            glyph[0] = 0x3F;
            glyph[1] = 0x40;
            glyph[2] = 0x40;
            glyph[3] = 0x3F;
            break;

        case 'V':
            glyph[0] = 0x1F;
            glyph[1] = 0x20;
            glyph[2] = 0x40;
            glyph[3] = 0x20;
            glyph[4] = 0x1F;
            break;

        case 'W':
            glyph[0] = 0x7F;
            glyph[1] = 0x20;
            glyph[2] = 0x18;
            glyph[3] = 0x20;
            glyph[4] = 0x7F;
            break;

        case 'X':
            glyph[0] = 0x63;
            glyph[1] = 0x14;
            glyph[2] = 0x08;
            glyph[3] = 0x14;
            glyph[4] = 0x63;
            break;

        case 'Y':
            glyph[0] = 0x07;
            glyph[1] = 0x08;
            glyph[2] = 0x70;
            glyph[3] = 0x08;
            glyph[4] = 0x07;
            break;

        case '0':
            glyph[0] = 0x3E;
            glyph[1] = 0x45;
            glyph[2] = 0x49;
            glyph[3] = 0x51;
            glyph[4] = 0x3E;
            break;

        case '1':
            glyph[0] = 0x00;
            glyph[1] = 0x42;
            glyph[2] = 0x7F;
            glyph[3] = 0x40;
            break;

        case '2':
            glyph[0] = 0x42;
            glyph[1] = 0x61;
            glyph[2] = 0x51;
            glyph[3] = 0x49;
            glyph[4] = 0x46;
            break;

        case '3':
            glyph[0] = 0x21;
            glyph[1] = 0x41;
            glyph[2] = 0x45;
            glyph[3] = 0x4B;
            glyph[4] = 0x31;
            break;

        case '4':
            glyph[0] = 0x18;
            glyph[1] = 0x14;
            glyph[2] = 0x12;
            glyph[3] = 0x7F;
            glyph[4] = 0x10;
            break;

        case '5':
            glyph[0] = 0x27;
            glyph[1] = 0x45;
            glyph[2] = 0x45;
            glyph[3] = 0x45;
            glyph[4] = 0x39;
            break;

        case '6':
            glyph[0] = 0x3E;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x49;
            glyph[4] = 0x30;
            break;

        case '7':
            glyph[0] = 0x01;
            glyph[1] = 0x71;
            glyph[2] = 0x09;
            glyph[3] = 0x05;
            glyph[4] = 0x03;
            break;

        case '8':
            glyph[0] = 0x36;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x49;
            glyph[4] = 0x36;
            break;

        case '9':
            glyph[0] = 0x06;
            glyph[1] = 0x49;
            glyph[2] = 0x49;
            glyph[3] = 0x49;
            glyph[4] = 0x3E;
            break;

        case ':':
            glyph[1] = 0x36;
            glyph[2] = 0x36;
            break;

        case '.':
            glyph[4] = 0x40;
            break;

        case '-':
            glyph[1] = 0x08;
            glyph[2] = 0x08;
            glyph[3] = 0x08;
            break;

        case '/':
            glyph[0] = 0x20;
            glyph[1] = 0x10;
            glyph[2] = 0x08;
            glyph[3] = 0x04;
            glyph[4] = 0x02;
            break;

        case '%':
            glyph[0] = 0x62;
            glyph[1] = 0x64;
            glyph[2] = 0x08;
            glyph[3] = 0x13;
            glyph[4] = 0x23;
            break;

        case 'x':
            glyph[1] = 0x36;
            glyph[2] = 0x08;
            glyph[3] = 0x36;
            break;

        case ' ':
        default:
            break;
    }
}


/*-----------------------------------------------------------
 * 显示一个字符
 *
 * x：0~122
 * y：必须为0、8、16、24、32、40、48、56
 *----------------------------------------------------------*/
void OLED_ShowChar(uint8_t x, uint8_t y, char chr)
{
    uint8_t glyph[5];
    uint8_t i;
    uint16_t index;

    if (x > 122U)
    {
        return;
    }

    if ((y & 0x07U) != 0U)
    {
        return;
    }

    if (y >= OLED_HEIGHT)
    {
        return;
    }

    OLED_GetGlyph(chr, glyph);

    index = ((uint16_t)(y / 8U) * OLED_WIDTH) + x;

    for (i = 0U; i < 5U; i++)
    {
        OLED_GRAM[index + i] = glyph[i];
    }

    /* 第6列空白，作为字符间距 */
    OLED_GRAM[index + 5U] = 0x00U;
}


/*-----------------------------------------------------------
 * 显示字符串
 *----------------------------------------------------------*/
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if (x > 122U)
        {
            break;
        }

        OLED_ShowChar(x, y, *str);

        x += 6U;

        str++;
    }
}
/*-----------------------------------------------------------
 * 显示两位小数的浮点数
 *
 * 示例：
 * 25.36
 * -1.27
 * 0.00
 *
 * 不使用printf浮点格式化
 *----------------------------------------------------------*/
void OLED_ShowFloat2(uint8_t x, uint8_t y, float value)
{
    char buffer[16];
    char digits[10];

    uint8_t pos = 0U;
    uint8_t digit_count = 0U;

    uint8_t i;

    uint32_t scaled;
    uint32_t integer_part;
    uint32_t fraction_part;

    float abs_value;

    /*-------------------------------------------------------
     * 处理正负号
     *------------------------------------------------------*/
    if (value < 0.0f)
    {
        buffer[pos++] = '-';
        abs_value = -value;
    }
    else
    {
        abs_value = value;
    }

    /*-------------------------------------------------------
     * 转换成放大100倍的整数
     *
     * 例如：
     * 25.36 -> 2536
     * -1.27 -> 127
     *------------------------------------------------------*/
    scaled = (uint32_t)(abs_value * 100.0f + 0.5f);

    integer_part = scaled / 100U;
    fraction_part = scaled % 100U;

    /*-------------------------------------------------------
     * 提取整数部分
     *------------------------------------------------------*/
    do
    {
        digits[digit_count++] =
            (char)('0' + (integer_part % 10U));

        integer_part /= 10U;

    } while (integer_part > 0U);

    /*-------------------------------------------------------
     * 逆序写入
     *------------------------------------------------------*/
    for (i = digit_count; i > 0U; i--)
    {
        buffer[pos++] = digits[i - 1U];
    }

    /* 小数点 */
    buffer[pos++] = '.';

    /* 小数第一位 */
    buffer[pos++] =
        (char)('0' + (fraction_part / 10U));

    /* 小数第二位 */
    buffer[pos++] =
        (char)('0' + (fraction_part % 10U));

    /* 字符串结束 */
    buffer[pos] = '\0';

    OLED_ShowString(x, y, buffer);
}

/*-----------------------------------------------------------
 * 显示 int16_t 类型整数
 *
 * 示例：
 * 123
 * -274
 * 0
 *----------------------------------------------------------*/
void OLED_ShowInt16(uint8_t x, uint8_t y, int16_t value)
{
    char buffer[8];
    uint8_t pos = 0U;

    uint16_t abs_value;

    /* 处理负数 */
    if (value < 0)
    {
        buffer[pos++] = '-';

        /*
         * 使用int32_t防止INT16_MIN取负时溢出
         */
        abs_value = (uint16_t)(-(int32_t)value);
    }
    else
    {
        abs_value = (uint16_t)value;
    }

    /*-------------------------------------------------------
     * 提取数字
     *------------------------------------------------------*/
    if (abs_value >= 10000U)
    {
        buffer[pos++] =
            (char)('0' + (abs_value / 10000U));

        abs_value %= 10000U;

        buffer[pos++] =
            (char)('0' + (abs_value / 1000U));

        abs_value %= 1000U;

        buffer[pos++] =
            (char)('0' + (abs_value / 100U));

        abs_value %= 100U;

        buffer[pos++] =
            (char)('0' + (abs_value / 10U));

        buffer[pos++] =
            (char)('0' + (abs_value % 10U));
    }
    else if (abs_value >= 1000U)
    {
        buffer[pos++] =
            (char)('0' + (abs_value / 1000U));

        abs_value %= 1000U;

        buffer[pos++] =
            (char)('0' + (abs_value / 100U));

        abs_value %= 100U;

        buffer[pos++] =
            (char)('0' + (abs_value / 10U));

        buffer[pos++] =
            (char)('0' + (abs_value % 10U));
    }
    else if (abs_value >= 100U)
    {
        buffer[pos++] =
            (char)('0' + (abs_value / 100U));

        abs_value %= 100U;

        buffer[pos++] =
            (char)('0' + (abs_value / 10U));

        buffer[pos++] =
            (char)('0' + (abs_value % 10U));
    }
    else if (abs_value >= 10U)
    {
        buffer[pos++] =
            (char)('0' + (abs_value / 10U));

        buffer[pos++] =
            (char)('0' + (abs_value % 10U));
    }
    else
    {
        buffer[pos++] =
            (char)('0' + abs_value);
    }

    buffer[pos] = '\0';

    OLED_ShowString(x, y, buffer);
}
