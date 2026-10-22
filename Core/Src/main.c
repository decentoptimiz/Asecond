/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "base.h"
#include "lcd.h"
#include "touch.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE   8192
#define DRAW_POINTS    240
#define GRID_COLOR     0x8410
#define SAMPLE_RATE    10000.0f
#define AVG_N          5
#define MUTATION_TH    0.08f

#define MODE_TRIG      0
#define MODE_VERT      1
#define MODE_HORI      2
#define MODE_CH_SEL    3   /* 新增：通道选择模式 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_dma_buf[ADC_BUF_SIZE];
extern DMA_HandleTypeDef hdma_adc1;

volatile uint8_t dma_ready = 0;
volatile uint8_t key0_pressed = 0;

float ch1_v[ADC_BUF_SIZE];
float ch2_v[ADC_BUF_SIZE];

float freq_buf1[AVG_N], freq_buf2[AVG_N];
float vpp_buf1[AVG_N],  vpp_buf2[AVG_N];
float avg_buf1[AVG_N],  avg_buf2[AVG_N];
uint8_t avg_idx = 0;
uint8_t avg_cnt = 0;

float last_f1_displayed = 0;
float last_f2_displayed = 0;

float   trig_level_ratio = 0.5f;
uint8_t trig_edge = 0;

const float v_div_table[3] = {0.01f, 0.1f, 1.0f};
uint8_t v_div_idx1 = 1;  // CH1 独立档位
uint8_t v_div_idx2 = 1;  // CH2 独立档位

const float t_div_table[3] = {0.001f, 0.01f, 0.1f};
uint8_t t_div_idx = 0;

int16_t y_offset = 0;

float last_vpp1 = 0, last_f1 = 0;
float last_vpp2 = 0, last_f2 = 0;

uint8_t current_mode = MODE_TRIG;
uint8_t trig_state = 0;

uint8_t active_ch = 1; // 当前操作通道：1=CH1, 2=CH2

float last_phase = 0;
uint8_t phase_init = 0;


// 档位映射：0(10mV) 用通道0；1(100mV) 用通道1；2(1V) 用通道2
    // CD4051 通道选择位 A(PA4), B(PA5), C(PA6)
    // 通道 0: C=0, B=0, A=0 (你的 8倍)
    // 通道 1: C=1, B=0, A=0 (你的 4倍)
    // 通道 2: C=0, B=1, A=0 (你的 1倍直通)
	//PA4 9 A PA5 10 B  PA6 11 C
	//PB12 9 A PB13 10 B  PB14 11 C
const float gain_val[3] = {40.0f, 4.0f, 1.0f}; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* 切换 CD4051 增益通道：ch=1或2, gain_idx=0,1,2 */
/* 切换 CD4051 增益通道：ch=1或2, gain_idx=0,1,2 */
void SetGain(uint8_t ch, uint8_t gain_idx)
{
    if (ch == 1) // CH1 控制 PA4(A), PA5(B), PA6(C)
    {
        if (gain_idx == 0) // 10mV档：8倍，通道0 (A=0, B=0, C=0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
        else if (gain_idx == 1) // 100mV档：4倍，通道1 (A=0, B=0, C=1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        }
        else // 1V档：1倍，通道2 (A=0, B=1, C=0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
    }
    else if (ch == 2) // CH2 控制 PB12(A), PB13(B), PB14(C)
    {
        if (gain_idx == 0) // 10mV档：8倍，通道0
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
        else if (gain_idx == 1) // 100mV档：4倍，通道1 (C=1)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        else // 1V档：1倍，通道2 (B=1)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
    }
}

void DrawGrid(void)
{
    uint16_t i;
    POINT_COLOR = GRID_COLOR;
    for(i = 0; i <= 8; i++)
    {
        LCD_DrawLine(0, i * (lcddev.height / 8),
                     lcddev.width - 1, i * (lcddev.height / 8));
        LCD_DrawLine(i * (lcddev.width / 8), 0,
                     i * (lcddev.width / 8), lcddev.height - 1);
    }
    POINT_COLOR = BLACK;
    LCD_DrawLine(0, lcddev.height / 2, lcddev.width - 1, lcddev.height / 2);
    LCD_DrawLine(lcddev.width / 2, 0, lcddev.width / 2, lcddev.height - 1);
}

void DrawWave(float *data, uint16_t start_idx, uint16_t show_len, uint16_t color, uint8_t v_idx)
{
    int i;
    POINT_COLOR = color;

    if(show_len < 2) show_len = 2;
    if(show_len > ADC_BUF_SIZE) show_len = ADC_BUF_SIZE;

    float v_full = 8.0f * v_div_table[v_idx];

    for(i = 0; i < DRAW_POINTS - 1; i++)
    {
        uint16_t x1 = i * (lcddev.width - 1) / (DRAW_POINTS - 1);
        uint16_t x2 = (i + 1) * (lcddev.width - 1) / (DRAW_POINTS - 1);

        uint16_t idx1 = (start_idx + (uint32_t)i * show_len / DRAW_POINTS) % ADC_BUF_SIZE;
        uint16_t idx2 = (start_idx + (uint32_t)(i + 1) * show_len / DRAW_POINTS) % ADC_BUF_SIZE;

        int16_t y1 = lcddev.height - 1 - (int16_t)(data[idx1] / v_full * (lcddev.height - 1)) - y_offset;
        int16_t y2 = lcddev.height - 1 - (int16_t)(data[idx2] / v_full * (lcddev.height - 1)) - y_offset;

        if(y1 < 0) y1 = 0;
        if(y1 >= lcddev.height) y1 = lcddev.height - 1;
        if(y2 < 0) y2 = 0;
        if(y2 >= lcddev.height) y2 = lcddev.height - 1;

        LCD_DrawLine(x1, (uint16_t)y1, x2, (uint16_t)y2);
    }
}

uint16_t FindTrigger(float *data, float max, float min, float ratio, uint8_t edge)
{
    int i;
    float level = min + (max - min) * ratio;
    for(i = 1; i < ADC_BUF_SIZE; i++)
    {
        if(edge == 0) { if(data[i-1] < level && data[i] >= level) return i; }
        else { if(data[i-1] > level && data[i] <= level) return i; }
    }
    return 0;
}

void CalcParams(float *data, float *vpp, float *avg, float *freq)
{
    int i;
    float max = data[0], min = data[0];
    double sum = 0;

    for(i = 0; i < ADC_BUF_SIZE; i++)
    {
        if(data[i] > max) max = data[i];
        if(data[i] < min) min = data[i];
        sum += data[i];
    }
    *vpp = max - min;
    *avg = (float)(sum / ADC_BUF_SIZE);

    float mid = (max + min) / 2.0f;
    float hyst = (max - min) * 0.15f;
    float up_th = mid + hyst;
    float dn_th = mid - hyst;

    uint16_t edges[128];
    int edge_cnt = 0;
    uint8_t state = 0;

    for(i = 0; i < ADC_BUF_SIZE; i++)
    {
        if(state == 0 && data[i] > up_th) { if(edge_cnt < 128) edges[edge_cnt++] = (uint16_t)i; state = 1; }
        else if(state == 1 && data[i] < dn_th) { state = 0; }
    }

    if(edge_cnt >= 2)
    {
        float avg_period = (float)(edges[edge_cnt - 1] - edges[0]) / (edge_cnt - 1);
        *freq = SAMPLE_RATE / avg_period;
    }
    else *freq = 0;
}

float FindFirstRisingEdgeInterp(float *data, float max, float min)
{
    float level = (max + min) / 2.0f;
    int i;
    for(i = 1; i < ADC_BUF_SIZE; i++)
    {
        if(data[i-1] < level && data[i] >= level)
        {
            float d1 = level - data[i-1];
            float d2 = data[i] - data[i-1];
            if(d2 > 0.0001f) return (float)(i - 1) + d1 / d2;
            return (float)i;
        }
    }
    return -1.0f;
}

float CalcPhaseDiff(float *ch1, float *ch2, float max1, float min1, float max2, float min2, float freq)
{
    float e1 = FindFirstRisingEdgeInterp(ch1, max1, min1);
    float e2 = FindFirstRisingEdgeInterp(ch2, max2, min2);
    if(e1 < 0 || e2 < 0) return -999.0f;

    float diff = e1 - e2;
    if(diff >  ADC_BUF_SIZE / 2.0f) diff -= ADC_BUF_SIZE;
    if(diff < -ADC_BUF_SIZE / 2.0f) diff += ADC_BUF_SIZE;

    float dt = diff / SAMPLE_RATE;
    float phase = dt * freq * 360.0f;
    while(phase >  180.0f) phase -= 360.0f;
    while(phase < -180.0f) phase += 360.0f;
    return phase;
}

float CalcHighRatio(float *data, float max, float min)
{
    float high_th = (max + min) / 2.0f + (max - min) * 0.4f;
    int high_cnt = 0;
    for(int i = 0; i < ADC_BUF_SIZE; i++) { if(data[i] > high_th) high_cnt++; }
    return (float)high_cnt / ADC_BUF_SIZE;
}

const char* IdentifyWave(float *data, float max, float min)
{
    float ratio = CalcHighRatio(data, max, min);
    if(ratio > 0.40f) return "Square";
    if(ratio > 0.10f) return "Sine";
    return "Triangle";
}

void AutoScale(float vpp, float freq, uint8_t *v_idx)
{
    float v_div_target = vpp / 4.5f;
    if(v_div_target <= 0.01f) *v_idx = 0;
    else if(v_div_target <= 0.1f) *v_idx = 1;
    else *v_idx = 2;

    if(freq > 1.0f)
    {
        float t_div_target = 4.0f / (8.0f * freq);
        if(t_div_target <= 0.001f) t_div_idx = 0;
        else if(t_div_target <= 0.01f) t_div_idx = 1;
        else t_div_idx = 2;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY0E4_Pin) key0_pressed = 1;
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);

  {
      GPIO_InitTypeDef gpio_init = {0};
      gpio_init.Pin  = GPIO_PIN_4;
      gpio_init.Mode = GPIO_MODE_IT_FALLING;
      gpio_init.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(GPIOE, &gpio_init);
      HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }

  LCD_Init();
  printf("LCD_Init done\r\n");

  LCD_Clear(WHITE);
  DrawGrid();
  
  /* 初始化硬件增益 */
  SetGain(1, v_div_idx1);
  SetGain(2, v_div_idx2);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  HAL_TIM_Base_Start(&htim8);
  __HAL_ADC_ENABLE(&hadc1);
  __HAL_ADC_ENABLE(&hadc2);

  DMA2_Stream0->CR = 0;
  DMA2->LIFCR = 0x3F;
  DMA2_Stream0->PAR  = (uint32_t)&ADC->CDR;
  DMA2_Stream0->M0AR = (uint32_t)adc_dma_buf;
  DMA2_Stream0->NDTR = ADC_BUF_SIZE;
  DMA2_Stream0->CR = (0  << 25) | (2 << 16) | (2 << 13) | (2 << 11)
                   | (1  << 10) | (0 <<  9) | (1 <<  8) | (0 <<  6)
                   | (1  <<  4) | (1 <<  0);

  ADC1->SR &= ~ADC_SR_OVR;
  ADC2->SR &= ~ADC_SR_OVR;
  ADC1->CR2 |= ADC_CR2_DMA;

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* KEY0：循环切换模式（4个模式） */
    if(key0_pressed)
    {
        key0_pressed = 0;
        HAL_Delay(20);
        if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            current_mode = (current_mode + 1) % 4;
            printf("[KEY0] Mode = %d\r\n", current_mode);
            while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET);
        }
    }

    /* KEY1：修改当前模式的值 */
    {
        static uint8_t key1_last = 1;
        uint8_t key1_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);

        if(key1_last == 1 && key1_now == 0)
        {
            HAL_Delay(20);
            if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
            {
                if(current_mode == MODE_TRIG)
                {
                    trig_state = (trig_state + 1) % 8;
                    switch(trig_state)
                    {
                        case 0: trig_edge = 0; trig_level_ratio = 0.2f; break;
                        case 1: trig_edge = 0; trig_level_ratio = 0.4f; break;
                        case 2: trig_edge = 0; trig_level_ratio = 0.6f; break;
                        case 3: trig_edge = 0; trig_level_ratio = 0.8f; break;
                        case 4: trig_edge = 1; trig_level_ratio = 0.2f; break;
                        case 5: trig_edge = 1; trig_level_ratio = 0.4f; break;
                        case 6: trig_edge = 1; trig_level_ratio = 0.6f; break;
                        case 7: trig_edge = 1; trig_level_ratio = 0.8f; break;
                    }
                }
                else if(current_mode == MODE_VERT)
                {
                    if(active_ch == 1) { v_div_idx1 = (v_div_idx1 + 1) % 3; SetGain(1, v_div_idx1); }
                    else { v_div_idx2 = (v_div_idx2 + 1) % 3; SetGain(2, v_div_idx2); }
                }
                else if(current_mode == MODE_HORI)
                {
                    t_div_idx = (t_div_idx + 1) % 3;
                }
                else if(current_mode == MODE_CH_SEL)
                {
                    active_ch = (active_ch == 1) ? 2 : 1; 
                }
                while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET);
            }
        }
        key1_last = key1_now;
    }

    /* KEY2：针对当前选中通道进行 AUTOSCALE */
    {
        static uint8_t key2_last = 1;
        uint8_t key2_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);

        if(key2_last == 1 && key2_now == 0)
        {
            HAL_Delay(20);
            if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
            {
                if(active_ch == 1)
                {
                    AutoScale(last_vpp1, last_f1, &v_div_idx1);
                    SetGain(1, v_div_idx1);
                }
                else
                {
                    AutoScale(last_vpp2, last_f2, &v_div_idx2);
                    SetGain(2, v_div_idx2);
                }
                while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET);
            }
        }
        key2_last = key2_now;
    }

    /* 波形刷新 */
    if(dma_ready)
    {
        dma_ready = 0;

        float g_cur1 = gain_val[v_div_idx1];
        float g_cur2 = gain_val[v_div_idx2];

        for(int i = 0; i < ADC_BUF_SIZE; i++)
        {
            ch1_v[i] = ((adc_dma_buf[i] & 0xFFFF) / 4095.0f) * 3.3f / g_cur1;
            ch2_v[i] = ((adc_dma_buf[i] >> 16)     / 4095.0f) * 3.3f / g_cur2;
        }

        float max1 = ch1_v[0], min1 = ch1_v[0];
        float max2 = ch2_v[0], min2 = ch2_v[0];
        for(int i = 0; i < ADC_BUF_SIZE; i++)
        {
            if(ch1_v[i] > max1) max1 = ch1_v[i]; if(ch1_v[i] < min1) min1 = ch1_v[i];
            if(ch2_v[i] > max2) max2 = ch2_v[i]; if(ch2_v[i] < min2) min2 = ch2_v[i];
        }

        uint16_t trig = FindTrigger(ch1_v, max1, min1, trig_level_ratio, trig_edge);

        float vpp1, avg1, freq1, vpp2, avg2, freq2;
        CalcParams(ch1_v, &vpp1, &avg1, &freq1);
        CalcParams(ch2_v, &vpp2, &avg2, &freq2);

        last_vpp1 = vpp1; last_f1 = freq1;
        last_vpp2 = vpp2; last_f2 = freq2;

        /* ====== 完整恢复的滑动平均滤波 ====== */
        uint8_t mutated1 = 0, mutated2 = 0;
        if(last_f1_displayed > 0 && freq1 > 0)
        {
            float delta = freq1 - last_f1_displayed;
            if(delta < 0) delta = -delta;
            if(delta / last_f1_displayed > MUTATION_TH) mutated1 = 1;
        }
        if(last_f2_displayed > 0 && freq2 > 0)
        {
            float delta = freq2 - last_f2_displayed;
            if(delta < 0) delta = -delta;
            if(delta / last_f2_displayed > MUTATION_TH) mutated2 = 1;
        }

        float f1_avg, f2_avg, v1_avg, v2_avg, a1_avg, a2_avg;

        if(mutated1 || mutated2)
        {
            for(int i = 0; i < AVG_N; i++)
            {
                freq_buf1[i] = freq1; freq_buf2[i] = freq2;
                vpp_buf1[i]  = vpp1;  vpp_buf2[i]  = vpp2;
                avg_buf1[i]  = avg1;  avg_buf2[i]  = avg2;
            }
            avg_idx = 1;
            avg_cnt = AVG_N;

            f1_avg = freq1; f2_avg = freq2;
            v1_avg = vpp1;  v2_avg = vpp2;
            a1_avg = avg1;  a2_avg = avg2;
						
        }
        else
        {
            freq_buf1[avg_idx] = freq1; freq_buf2[avg_idx] = freq2;
            vpp_buf1[avg_idx]  = vpp1;  vpp_buf2[avg_idx]  = vpp2;
            avg_buf1[avg_idx]  = avg1;  avg_buf2[avg_idx]  = avg2;
            avg_idx = (avg_idx + 1) % AVG_N;
            if(avg_cnt < AVG_N) avg_cnt++;

            float fs1=0, fs2=0, vs1=0, vs2=0, as1=0, as2=0;
            for(int i = 0; i < avg_cnt; i++)
            {
                fs1 += freq_buf1[i]; fs2 += freq_buf2[i];
                vs1 += vpp_buf1[i];  vs2 += vpp_buf2[i];
                as1 += avg_buf1[i];  as2 += avg_buf2[i];
            }
            f1_avg = fs1 / avg_cnt; f2_avg = fs2 / avg_cnt;
            v1_avg = vs1 / avg_cnt; v2_avg = vs2 / avg_cnt;
            a1_avg = as1 / avg_cnt; a2_avg = as2 / avg_cnt;
        }
        last_f1_displayed = f1_avg;
        last_f2_displayed = f2_avg;

				
				/* ★ 反向补偿逻辑 ★ */
        // 因为之前的 ch1_v 已经除过增益，算出的 v1_avg 已是真实电压
        // 先乘回增益 -> 减 50mV -> 再除以增益
        v1_avg = (v1_avg * g_cur1 - 0.05f) / g_cur1;
        a1_avg = (a1_avg * g_cur1 - 0.05f) / g_cur1;
        
        v2_avg = (v2_avg * g_cur2 - 0.05f) / g_cur2;
        a2_avg = (a2_avg * g_cur2 - 0.05f) / g_cur2;

        // 防止出现负数
        if (v1_avg < 0) v1_avg = 0.0f;
        if (a1_avg < 0) a1_avg = 0.0f;
        if (v2_avg < 0) v2_avg = 0.0f;
        if (a2_avg < 0) a2_avg = 0.0f;

        int show_len = (int)(8.0f * t_div_table[t_div_idx] * SAMPLE_RATE);
				
				

        LCD_Clear(WHITE);
        DrawGrid();
        DrawWave(ch1_v, trig, show_len, RED, v_div_idx1);
        DrawWave(ch2_v, trig, show_len, BLUE, v_div_idx2);

        char buf[80];
        POINT_COLOR = BLACK;
        BACK_COLOR  = WHITE;

        sprintf(buf, "CH1: %5.0fHz Vpp:%.3fV", f1_avg, v1_avg);
        LCD_ShowString(5, 5, 240, 12, 12, (uint8_t *)buf);
        sprintf(buf, "     Vavg:%.3fV", a1_avg);
        LCD_ShowString(5, 18, 240, 12, 12, (uint8_t *)buf);

        sprintf(buf, "CH2: %5.0fHz Vpp:%.3fV", f2_avg, v2_avg);
        LCD_ShowString(5, 32, 240, 12, 12, (uint8_t *)buf);
        sprintf(buf, "     Vavg:%.3fV", a2_avg);
        LCD_ShowString(5, 45, 240, 12, 12, (uint8_t *)buf);

        float phase = CalcPhaseDiff(ch1_v, ch2_v, max1, min1, max2, min2, f1_avg);
        if(phase > -900.0f)
        {
            if(!phase_init) { last_phase = phase; phase_init = 1; }
            else {
                float diff = phase - last_phase;
                while(diff >  180.0f) diff -= 360.0f;
                while(diff < -180.0f) diff += 360.0f;
                if(fabsf(diff) > 8.0f) last_phase = phase;
                else last_phase = last_phase * 0.6f + phase * 0.4f;
            }
            sprintf(buf, "Phase: %+.1f deg", last_phase);
        }
        else sprintf(buf, "Phase: ---");
        LCD_ShowString(5, 60, 240, 12, 12, (uint8_t *)buf);

        const char *wave_type = IdentifyWave(ch1_v, max1, min1);
        const char *mode_name = "TRIG";
        if(current_mode == MODE_VERT) mode_name = "VERT";
        else if(current_mode == MODE_HORI) mode_name = "HORI";
        else if(current_mode == MODE_CH_SEL) mode_name = "CH_SEL";

        sprintf(buf, "[%s] Active:CH%d", mode_name, active_ch);
        LCD_ShowString(5, 73, 240, 12, 12, (uint8_t *)buf);

        sprintf(buf, "V1:%d V2:%d T:%d", v_div_idx1, v_div_idx2, t_div_idx);
        LCD_ShowString(5, 86, 240, 12, 12, (uint8_t *)buf);

        printf("CH1: F=%.1fHz Vpp=%.3fV | CH2: F=%.1fHz Vpp=%.3fV | Phase=%.1f Wave=%s Mode=%s CH=%d\r\n",
               f1_avg, v1_avg, f2_avg, v2_avg, last_phase, wave_type, mode_name, active_ch);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
