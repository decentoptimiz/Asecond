// base.c 文件
#include "base.h"

static u8 fac_us = 0; // us延时倍乘数

// 初始化延迟函数
// SYSTICK的时钟固定为HCLK时钟的1/8
// SYSCLK:系统时钟
void delay_init(u8 SYSCLK)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // SysTick频率为HCLK
    fac_us = SYSCLK;
}

void delay_ns(u8 t)
{
    do
    {
        ;
    } while (--t);
}

void delay_us(u32 nus)
{
    u32 ticks;
    u32 told, tnow, tcnt = 0;
    u32 reload = SysTick->LOAD; // LOAD的值
    ticks = nus * fac_us;       // 需要的节拍数
    told = SysTick->VAL;        // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出
        }
    }
}

// 延时nms
// nms:要延时的ms数
void delay_ms(u16 nms)
{
    u32 i;
    for (i = 0; i < nms; i++)
        delay_us(1000);
}

// 按键扫描函数
// 只保留 KEY0（PE4），KEY1/KEY2 已删除
// 返回 1 表示 KEY0 按下，0 表示未按下
u8 KEY_Scan(u8 mode)
{
    static u8 key_up = 1; // 按键按松开标志

    if (mode)
        key_up = 1;

    if (key_up && (KEY0E4 == 0))
    {
        HAL_Delay(10); // 去抖动
        key_up = 0;
        if (KEY0E4 == 0)
            return 1;
    }
    else if (KEY0E4 == 1)
    {
        key_up = 1;
    }

    return 0;
}
