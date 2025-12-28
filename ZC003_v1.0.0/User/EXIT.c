#include "EXIT.h"

#include "delay.h"
#include "lcd.h"
#include "lcd_init.h"
#include "measure.h"

extern uint32_t Freq[FREQ_LENGTH]; //测量到的频率
extern volatile uint16_t Freq_cnt; //记录测到第几个频率
extern volatile uint8_t FreqMeasure_Flag; //记录是第几次测量

// 全局变量定义
volatile uint32_t gEdgeCount = 0;      // 边沿计数器
volatile uint32_t gFrequency = 0;      // 频率存储
volatile uint32_t gMeasureInterval = 1;// 测量间隔（秒），需与sysconfig配置对应

// 中断初始化
void Interrupt_Init(void)
{
    NVIC_ClearPendingIRQ(KEY_INT_IRQN); //清除中断标志位
    NVIC_EnableIRQ(KEY_INT_IRQN); //使能中断

    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

//GPIO的中断服务函数
void GROUP1_IRQHandler(void)
{
    uint32_t GPIO_IIDX = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1); //获取中断标志位
    switch (GPIO_IIDX)
    {
        case INPUT_INT_IIDX: //检查是否是频率测量输入引脚的中断
            //检查引脚电平变化
            if(DL_GPIO_readPins(INPUT_PORT, INPUT_PIN_IN_PIN) > 0)
            {
                //边沿计数器自增
                gEdgeCount++;
            }
            break;

        case KEY_INT_IIDX: //若为开始测量按键
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_KEY1_PIN) > 0) //PA18
            {
                delay_ms(20);

                LCD_Fill(0, 190, LCD_W, LCD_H, WHITE);
                LCD_ShowString(30, 190, (u8 *)"Measure start!", GREEN, WHITE, 32, 1);

                FreqMeasure_Flag = 0;
                StartMeasure();

                while (DL_GPIO_readPins(KEY_PORT, KEY_PIN_KEY1_PIN)); //等待变为低电平
                delay_ms(20);
            }
            break;

        default:
            break;
    }
}

// 定时器中断处理函数（用于定期计算频率）
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) 
    {
        case DL_TIMER_IIDX_ZERO:
            //计算频率
            gFrequency = gEdgeCount / gMeasureInterval;
            //重置边沿计数器
            gEdgeCount = 0;
            //记录数据
            Freq[Freq_cnt] = gFrequency;
            Freq_cnt ++;

            if ((Freq_cnt >= FREQ_LENGTH) && (FreqMeasure_Flag == 0))
            {
                FreqMeasure_Flag ++;
                Freq_cnt = 0;
                StopMeasure(); //停止测量
            }
            else if ((Freq_cnt >= FREQ_LENGTH) && (FreqMeasure_Flag > 0))
            {
                Freq_cnt = 0;
                StopMeasure2(); //停止测量
            }

            break;
        default:
            break;
    }
}