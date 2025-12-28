#include "measure.h"

#include "delay.h"
#include "lcd.h"
#include "lcd_init.h"
#include "EXIT.h"
#include "adf4002.h"

extern uint32_t Freq[FREQ_LENGTH]; //测量到的频率
extern volatile uint16_t Freq_cnt; //记录测到第几个频率
extern volatile uint8_t FreqMeasure_Flag; //记录是第几次测量

volatile uint32_t Freq_Final = 0; //最终频率

//初始化分频
void Measure_Init(void)
{
    // RDivideTest(2); // 输入1分频（1--8191）----REFIN低频
    NDivideTest(1000); // 输入2分频（1--16383） ----RFIN
}

//开始测量
void StartMeasure(void)
{
    if (FreqMeasure_Flag == 0)
        NDivideTest(1000); // 输入2分频（1--16383） ----RFIN
    else if (FreqMeasure_Flag > 0)
        NDivideTest(2); // 输入2分频（1--16383） ----RFIN

    NVIC_ClearPendingIRQ(INPUT_INT_IRQN); //清除中断标志位
    NVIC_EnableIRQ(INPUT_INT_IRQN); //使能中断

    DL_TimerG_startCounter(TIMER_0_INST); //打开定时器
    Freq_cnt = 0;
}

//停止测量
void StopMeasure(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);
    NVIC_DisableIRQ(INPUT_INT_IRQN);

    Freq_Final = Result_Calculate(Freq, FREQ_LENGTH); //数据处理

    if (Freq_Final > 500)
    {
        // Freq_Final = Freq_Final * 1000;
        LCD_Fill(110, 70, 160, 87, WHITE);
        LCD_ShowInt32Num(110, 70, Freq_Final, 9, BLACK, WHITE, 16);
        LCD_Fill(0, 190, LCD_W, LCD_H, WHITE);
        LCD_ShowString(30, 190, (u8 *)"Measure done!", GREEN, WHITE, 32, 1);
        
        FreqMeasure_Flag = 0;
    }
    else
    {
        StartMeasure();
    }
}

//停止第二次测量
void StopMeasure2(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);
    NVIC_DisableIRQ(INPUT_INT_IRQN);

    Freq_Final = Result_Calculate(Freq, FREQ_LENGTH); //数据处理

    Freq_Final = Freq_Final * 2;
    LCD_Fill(110, 70, 160, 87, WHITE);
    LCD_ShowInt32Num(110, 70, Freq_Final, 9, BLACK, WHITE, 16);
    LCD_Fill(0, 190, LCD_W, LCD_H, WHITE);
    LCD_ShowString(30, 190, (u8 *)"Measure done!", GREEN, WHITE, 32, 1);

    FreqMeasure_Flag = 0;
}

//比较函数，用于排序
int compare(const void *a, const void *b)
{
    return (*(uint16_t*)a - *(uint16_t*)b);
}

/**
 * @brief 计算统计结果
 * @param IndexDiff 未排序的原始数组
 * @param len 数组长度
 * @return uint16_t 最终结果（众数或中位数）
 */
uint32_t Result_Calculate(uint32_t *IndexDiff, uint16_t len)
{
    if (len == 0) return 0; //空数组保护

    /*---- 数组排序 ----*/
    uint32_t *sorted = malloc(len * sizeof(uint32_t)); //分配内存
    memcpy(sorted, IndexDiff, len * sizeof(uint32_t)); //复制数组
    qsort(sorted, len, sizeof(uint32_t), compare); //快速排序

    /*---- 计算中位数 ----*/
    float median;
    if (len % 2 == 0)
    {
        median = (sorted[len / 2 - 1] + sorted[len / 2]) / 2.0f;
    }
    else
    {
        median = sorted[len / 2];
    }

    /*---- 寻找众数 ----*/
    uint32_t current_val = sorted[0]; //当前的数
    uint16_t current_cnt = 1; //当前的数出现次数
    uint16_t max_cnt = 1; //众数出现次数
    uint32_t mode = current_val; //众数

    for (uint16_t i = 1; i < len; i++)
    {
        if (sorted[i] == current_val)
        {
            current_cnt ++;
        }
        else
        {
            current_val = sorted[i];
            current_cnt = 1;
        }

        //更新众数（相同次数时取较小值）
        if ((current_cnt > max_cnt) || (current_cnt == max_cnt && current_val < mode))
        {
            max_cnt = current_cnt;
            mode = current_val;
        }
    }

    /*---- 决定最终输出 ----*/
    uint32_t final_result;
    if (max_cnt > 1) //众数出现大于固定次数
    {
        final_result = mode; //有真实众数
    }
    else
    {
        final_result = (uint32_t)(median + 0.5f); //四舍五入取中位数
    }

    free(sorted);
    return final_result;
}
