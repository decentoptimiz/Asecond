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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* 大彩串口屏 HMI 驱动（触摸人机交互） */
#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "hmi_text.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE   8192
#define DRAW_POINTS    400      /* 8div × 50点/div = 400，满足任务书水平分辨率要求 */
#define SAMPLE_RATE    10000.0f
#define AVG_N          5
#define MUTATION_TH    0.08f

#define MODE_TRIG      0
#define MODE_VERT      1
#define MODE_HORI      2
#define MODE_CH_SEL    3   /* ������ͨ��ѡ��ģʽ */
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
uint8_t v_div_idx1 = 1;  // CH1 ������λ
uint8_t v_div_idx2 = 1;  // CH2 ������λ

const float t_div_table[3] = {0.001f, 0.01f, 0.1f};
uint8_t t_div_idx = 0;

float last_vpp1 = 0, last_f1 = 0;
float last_vpp2 = 0, last_f2 = 0;

uint8_t current_mode = MODE_TRIG;
uint8_t trig_state = 0;

uint8_t active_ch = 1; // ��ǰ����ͨ����1=CH1, 2=CH2

float last_phase = 0;
uint8_t phase_init = 0;

	//PA4 9 A PA5 10 B  PA6 11 C
	//PB12 9 A PB13 10 B  PB14 11 C
/* ===== 模拟前端参数 ===== 
 * ADC 参考电压（VDDA）：所有电压换算都以它为基准 
 * 前端直流偏置：ADC 收到的是「信号 + 偏置」，必须减掉才是真实电压 
 *   +5V--10k--+--10k--GND  -> 2.5V     （当前电路图） 
 *   +5V--20k--+--10k--GND  -> 1.667V   （推荐：留给正半周的余量更大） 
 * 注意：偏置越靠近 3.3V，正半周可用摆幅越小；2.5V 时只剩 0.8V 余量 */
#define ADC_VREF      3.3f
#define ADC_BIAS_V    2.5f

/* 增益标定表 [通道][档位]，通道 0=CH1 1=CH2，档位 0=10mV/div 1=100mV/div 2=1V/div
 * 理论值 40 / 4 / 1（Rf = 390k / 30k / 0 欧，Rg = 10k）
 * CD4051 导通电阻串在反馈回路里，×1 档实际增益会偏大 2%~4%，务必实测后填这里
 * 标定方法：输入已知峰峰值的正弦（如 1kHz、1.000Vpp），切到该档位读屏上「峰峰值」Vmeas，
 *           则 新值 = 当前值 × Vmeas / Vtrue */
const float gain_cal[2][3] = {
    {40.0f, 4.0f, 1.0f},   /* CH1 */
    {40.0f, 4.0f, 1.0f},   /* CH2 */
};

/* ===== 串口屏 HMI =====
 * 屏端画面/控件 ID 约定（以 VisualTFT 工程 Screen0.tft 为准）：
 *   画面 0：主界面
 *   曲线控件：GRAPH1(ID=1) 单控件双通道
 *             通道0 = CH1 波形（黄），通道1 = CH2 波形（品红）
 *             采样点数 400、采样深度 2 Bytes、显示范围 0~32767
 *   文本控件：3=CH1频率 4=CH1峰峰值 5=CH1平均值
 *             6=CH2频率 7=CH2峰峰值 8=CH2平均值
 *             9=相位差 10=波形类型 11=模式/通道 12=档位/边沿
 *   按钮控件：13=模式切换 14=通道选择 15=CH1档位+ 16=CH1档位-
 *             17=CH2档位+ 18=CH2档位- 19=时基+ 20=时基-
 *             21=触发边沿 22=触发电平+ 23=触发电平- 24=AUTOSCALE
 */
#define HMI_SCREEN      0
#define HMI_GRAPH       1   /* 唯一的曲线控件 ID */
#define HMI_GRAPH_CH1   0   /* 曲线控件内的通道号：CH1 */
#define HMI_GRAPH_CH2   1   /* 曲线控件内的通道号：CH2 */
#define HMI_TXT_F1      3
#define HMI_TXT_VPP1    4
#define HMI_TXT_AVG1    5
#define HMI_TXT_F2      6
#define HMI_TXT_VPP2    7
#define HMI_TXT_AVG2    8
#define HMI_TXT_PHASE   9
#define HMI_TXT_WAVE    10
#define HMI_TXT_MODE    11
#define HMI_TXT_DIV     12

uint8_t cmd_buffer[CMD_MAX_SIZE];              /* 串口屏指令接收缓存 */
uint8_t hmi_wave_buf[DRAW_POINTS * 2];         /* 曲线数据缓存（16bit，高字节在前，共 480 字节） */
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

/* �л� CD4051 ����ͨ����ch=1��2, gain_idx=0,1,2 */
/* �л� CD4051 ����ͨ����ch=1��2, gain_idx=0,1,2 */
void SetGain(uint8_t ch, uint8_t gain_idx)
{
    if (ch == 1) // CH1 ���� PA4(A), PA5(B), PA6(C)
    {
        if (gain_idx == 0) // 10mV����8����ͨ��0 (A=0, B=0, C=0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
        else if (gain_idx == 1) // 100mV����4����ͨ��1 (A=0, B=0, C=1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        }
        else // 1V����1����ͨ��2 (A=0, B=1, C=0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
    }
    else if (ch == 2) // CH2 ���� PB12(A), PB13(B), PB14(C)
    {
        if (gain_idx == 0) // 10mV����8����ͨ��0
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
        else if (gain_idx == 1) // 100mV����4����ͨ��1 (C=1)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        else // 1V����1����ͨ��2 (B=1)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
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

/* 波形识别：用「峰均比（crest factor）」区分，比高电平占比法可靠得多
 *   峰均比 = 峰值 / 有效值(交流)
 *   方波   ≈ 1.00     （峰值=有效值）
 *   正弦波 ≈ 1.414    （峰值 = 有效值 × √2）
 *   三角波 ≈ 1.732    （峰值 = 有效值 × √3）
 * 阈值取两两中间值：方波/正弦 1.20，正弦/三角 1.58 */
static const char* IdentifyWave(float *data, float max, float min)
{
    int i;
    double sum = 0.0, sq = 0.0;
    float  avg, rms, peak, crest;

    for(i = 0; i < ADC_BUF_SIZE; i++) sum += data[i];
    avg = (float)(sum / ADC_BUF_SIZE);

    for(i = 0; i < ADC_BUF_SIZE; i++)
    {
        double d = (double)data[i] - (double)avg;
        sq += d * d;
    }
    rms = (float)sqrt(sq / ADC_BUF_SIZE);

    if(rms < 1e-5f) return HT_NONE;                  /* 无信号 */

    peak  = (max - min) * 0.5f;
    crest = peak / rms;

    if(crest < 1.20f) return HT_SQUARE;              /* 方波 */
    if(crest < 1.58f) return HT_SINE;                /* 正弦波 */
    return HT_TRI;                                   /* 三角波 */
}

/* AUTOSCALE：根据当前信号自动选档
 *  - 两通道使用同一垂直档位（保证波形比例一致、便于比较）
 *  - 触发电平自动置于信号中点 50%
 *  - 频率无效时保持原时基不动
 */
void AutoScaleAll(void)
{
    float vpp  = (last_vpp1 > last_vpp2) ? last_vpp1 : last_vpp2;   /* 取较大者，防止任一路超屏 */
    float freq = (last_f1 > 1.0f) ? last_f1 : last_f2;
    uint8_t idx;

    /* 垂直档位：让波形占约 4.5 格 */
    if(vpp > 0.002f)
    {
        float v_div_target = vpp / 4.5f;
        if(v_div_target <= 0.01f)     idx = 0;    /* 10mV/div  */
        else if(v_div_target <= 0.1f) idx = 1;    /* 100mV/div */
        else                          idx = 2;    /* 1V/div    */

        v_div_idx1 = idx;
        v_div_idx2 = idx;
        SetGain(1, idx);
        SetGain(2, idx);
    }

    /* 水平时基：让屏幕显示约 4 个周期 */
    if(freq >= 10.0f && freq <= 1100.0f)
    {
        float t_div_target = 4.0f / (8.0f * freq);
        if(t_div_target <= 0.001f)      t_div_idx = 0;   /* 1ms/div   */
        else if(t_div_target <= 0.01f)  t_div_idx = 1;   /* 10ms/div  */
        else                            t_div_idx = 2;   /* 100ms/div */
    }

    /* 触发电平自动置于信号中点，保证稳定触发 */
    trig_level_ratio = 0.5f;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY0E4_Pin) key0_pressed = 1;
}

/* ===================== 串口屏 HMI 触摸交互 ===================== */

/* 从源数据中取第 i 个显示点：
 *   show_len <= 240 时用线性插值（上采样，消除阶梯感）
 *   show_len >  240 时用区间平均（下采样，抑制 ADC 噪声与混叠） */
static float HMI_SampleAt(float *data, uint16_t start, int show_len, int i)
{
    if(show_len <= DRAW_POINTS)
    {
        int span = (show_len > 1) ? (show_len - 1) : 1;
        float pos = (float)i * (float)span / (float)(DRAW_POINTS - 1);
        int   k   = (int)pos;
        float fr  = pos - (float)k;
        uint16_t i1 = (uint16_t)((start + k)     % ADC_BUF_SIZE);
        uint16_t i2 = (uint16_t)((start + k + 1) % ADC_BUF_SIZE);
        return data[i1] * (1.0f - fr) + data[i2] * fr;
    }
    else
    {
        int a = (int)((float)i       * (float)show_len / (float)DRAW_POINTS);
        int b = (int)((float)(i + 1) * (float)show_len / (float)DRAW_POINTS);
        int k, n = 0;
        float sum = 0.0f;
        if(b <= a) b = a + 1;
        if(b > show_len) b = show_len;
        for(k = a; k < b; k++)
        {
            sum += data[(uint16_t)((start + k) % ADC_BUF_SIZE)];
            n++;
        }
        return (n > 0) ? (sum / (float)n) : 0.0f;
    }
}

/* 曲线数据上限（屏端「曲线顶部值」需与此一致） */
#define HMI_WAVE_MAX   32767.0f

/* 把 CH1/CH2 波形定点化后发送到串口屏曲线控件
 * 屏端配置：单控件双通道、采样深度 2 Bytes、显示范围 0~32767
 * 数据格式：每个采样点 2 字节，高字节在前
 * 减去直流偏置让波形居中，并按实际峰峰值自适应缩放，让波形铺满曲线区 */
static void HMI_UpdateWave(uint16_t trig, int show_len,
                           float avg1, float avg2, float vpp1, float vpp2)
{
    int i;
    /* 按峰峰值缩放，留 25% 边距；信号太小时给个最小量程防止噪声放大 */
    float scale1 = (vpp1 > 0.01f) ? (vpp1 * 1.25f) : 0.5f;
    float scale2 = (vpp2 > 0.01f) ? (vpp2 * 1.25f) : 0.5f;

    /* CH1 -> 曲线控件通道0 */
    GraphChannelDataClear(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH1);
    for(i = 0; i < DRAW_POINTS; i++)
    {
        float px = ((HMI_SampleAt(ch1_v, trig, show_len, i) - avg1) / scale1 + 0.5f) * HMI_WAVE_MAX;
        if(px < 0.0f) px = 0.0f;
        if(px > HMI_WAVE_MAX) px = HMI_WAVE_MAX;
        hmi_wave_buf[2 * i]     = (uint8_t)(((uint16_t)px) >> 8);   /* 高字节 */
        hmi_wave_buf[2 * i + 1] = (uint8_t)(((uint16_t)px) & 0xFF); /* 低字节 */
    }
    GraphChannelDataAdd(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH1, hmi_wave_buf, DRAW_POINTS * 2);

    /* CH2 -> 曲线控件通道1 */
    GraphChannelDataClear(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH2);
    for(i = 0; i < DRAW_POINTS; i++)
    {
        float px = ((HMI_SampleAt(ch2_v, trig, show_len, i) - avg2) / scale2 + 0.5f) * HMI_WAVE_MAX;
        if(px < 0.0f) px = 0.0f;
        if(px > HMI_WAVE_MAX) px = HMI_WAVE_MAX;
        hmi_wave_buf[2 * i]     = (uint8_t)(((uint16_t)px) >> 8);   /* 高字节 */
        hmi_wave_buf[2 * i + 1] = (uint8_t)(((uint16_t)px) & 0xFF); /* 低字节 */
    }
    GraphChannelDataAdd(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH2, hmi_wave_buf, DRAW_POINTS * 2);
}

/* 更新串口屏上的模式/通道/档位状态文本 */
static void HMI_UpdateStatus(void)
{
    char buf[64];
    const char *mode_name = HT_TRIG;
    const char *edge_name = HT_RISE;
    const char *div_name[3] = {"10mV", "100mV", "1V"};
    const char *tdiv_name[3] = {"1ms", "10ms", "100ms"};

    if(current_mode == MODE_VERT) mode_name = HT_VERT;
    else if(current_mode == MODE_HORI) mode_name = HT_HORI;
    else if(current_mode == MODE_CH_SEL) mode_name = HT_CHSEL;
    if(trig_edge == 1) edge_name = HT_FALL;

    /* 模式 + 当前通道 + 触发边沿，如「触发 通道1 上升」 */
    sprintf(buf, "%s %s %s", mode_name, (active_ch == 1) ? HT_CH1 : HT_CH2, edge_name);
    SetTextValue(HMI_SCREEN, HMI_TXT_MODE, (uchar *)buf);

    /* 垂直档位与时基：两通道同档时合并显示，避免超出控件宽度被截断
     *   同档： "100mV 1ms"
     *   不同： "100mV/1V 1ms" */
    if(v_div_idx1 == v_div_idx2)
        sprintf(buf, "%s %s", div_name[v_div_idx1], tdiv_name[t_div_idx]);
    else
        sprintf(buf, "%s/%s %s", div_name[v_div_idx1], div_name[v_div_idx2], tdiv_name[t_div_idx]);
    SetTextValue(HMI_SCREEN, HMI_TXT_DIV, (uchar *)buf);
}

/* 更新串口屏上的测量参数文本 */
static void HMI_UpdateText(float f1, float vpp1, float avg1,
                           float f2, float vpp2, float avg2,
                           float phase, const char *wave_type)
{
    char buf[64];

    sprintf(buf, HT_FREQ "%.0fHz", f1);
    SetTextValue(HMI_SCREEN, HMI_TXT_F1, (uchar *)buf);
    sprintf(buf, HT_VPP "%.3fV", vpp1);
    SetTextValue(HMI_SCREEN, HMI_TXT_VPP1, (uchar *)buf);
    sprintf(buf, HT_AVG "%.3fV", avg1);
    SetTextValue(HMI_SCREEN, HMI_TXT_AVG1, (uchar *)buf);

    sprintf(buf, HT_FREQ "%.0fHz", f2);
    SetTextValue(HMI_SCREEN, HMI_TXT_F2, (uchar *)buf);
    sprintf(buf, HT_VPP "%.3fV", vpp2);
    SetTextValue(HMI_SCREEN, HMI_TXT_VPP2, (uchar *)buf);
    sprintf(buf, HT_AVG "%.3fV", avg2);
    SetTextValue(HMI_SCREEN, HMI_TXT_AVG2, (uchar *)buf);

    if(phase > -900.0f) sprintf(buf, HT_PHASE "%+.1f", phase);
    else                sprintf(buf, HT_PHASE HT_NONE);
    SetTextValue(HMI_SCREEN, HMI_TXT_PHASE, (uchar *)buf);

    sprintf(buf, HT_WAVE "%s", wave_type);
    SetTextValue(HMI_SCREEN, HMI_TXT_WAVE, (uchar *)buf);

    HMI_UpdateStatus();
}

/* 串口屏指令处理入口（在主循环中轮询调用） */
static void HMI_Process(void)
{
    qsize size = queue_find_cmd(cmd_buffer, CMD_MAX_SIZE);
    if(size > 0)
    {
        ProcessMessage((PCTRL_MSG)cmd_buffer, size);
    }
}

/* 消息分发：把串口屏上传的指令转换为业务回调 */
void ProcessMessage(PCTRL_MSG msg, uint16 size)
{
    uint8  cmd_type     = msg->cmd_type;
    uint8  ctrl_msg     = msg->ctrl_msg;
    uint8  control_type = msg->control_type;
    uint16 screen_id    = PTR2U16(&msg->screen_id);
    uint16 control_id   = PTR2U16(&msg->control_id);

    switch(cmd_type)
    {
    case NOTIFY_TOUCH_PRESS:                                    /* 触摸按下 */
    case NOTIFY_TOUCH_RELEASE:                                  /* 触摸松开 */
        NotifyTouchXY(cmd_buffer[1], PTR2U16(cmd_buffer + 2), PTR2U16(cmd_buffer + 4));
        break;

    case NOTIFY_CONTROL:                                        /* 控件事件 */
        if(ctrl_msg == MSG_GET_CURRENT_SCREEN)                  /* 画面切换通知 */
        {
            NotifyScreen(screen_id);
        }
        else
        {
            switch(control_type)
            {
            case kCtrlButton:                                   /* 按钮 */
                NotifyButton(screen_id, control_id, msg->param[1]);
                break;
            case kCtrlText:                                     /* 文本（键盘输入回传） */
                NotifyText(screen_id, control_id, msg->param);
                break;
            default:
                break;
            }
        }
        break;

    default:
        break;
    }
}

/* 画面切换通知 */
void NotifyScreen(uint16 screen_id)
{
    /* 进入主界面时，清空两条曲线通道 */
    if(screen_id == HMI_SCREEN)
    {
        GraphChannelDataClear(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH1);
        GraphChannelDataClear(HMI_SCREEN, HMI_GRAPH, HMI_GRAPH_CH2);
    }
}

/* 触摸坐标事件（预留） */
void NotifyTouchXY(uint8 press, uint16 x, uint16 y)
{
}

/* 按钮控件通知：实现功能切换（与原按键逻辑一致） */
void NotifyButton(uint16 screen_id, uint16 control_id, uint8 state)
{
    if(state == 0) return;                                      /* 只响应按下 */

    switch(control_id)
    {
    case 13:                                                    /* 模式切换 */
        current_mode = (current_mode + 1) % 4;
        break;
    case 14:                                                    /* 通道选择 */
        active_ch = (active_ch == 1) ? 2 : 1;
        break;
    case 15:                                                    /* CH1 垂直档位+ */
        if(v_div_idx1 < 2) { v_div_idx1++; SetGain(1, v_div_idx1); }
        break;
    case 16:                                                    /* CH1 垂直档位- */
        if(v_div_idx1 > 0) { v_div_idx1--; SetGain(1, v_div_idx1); }
        break;
    case 17:                                                    /* CH2 垂直档位+ */
        if(v_div_idx2 < 2) { v_div_idx2++; SetGain(2, v_div_idx2); }
        break;
    case 18:                                                    /* CH2 垂直档位- */
        if(v_div_idx2 > 0) { v_div_idx2--; SetGain(2, v_div_idx2); }
        break;
    case 19:                                                    /* 水平时基+ */
        if(t_div_idx < 2) t_div_idx++;
        break;
    case 20:                                                    /* 水平时基- */
        if(t_div_idx > 0) t_div_idx--;
        break;
    case 21:                                                    /* 触发边沿切换 */
        trig_edge = (trig_edge == 0) ? 1 : 0;
        break;
    case 22:                                                    /* 触发电平+ */
        if(trig_level_ratio < 0.95f) trig_level_ratio += 0.1f;
        break;
    case 23:                                                    /* 触发电平- */
        if(trig_level_ratio > 0.05f) trig_level_ratio -= 0.1f;
        break;
    case 24:                                                    /* AUTOSCALE */
        AutoScaleAll();
        break;
    default:
        break;
    }

    HMI_UpdateStatus();                                         /* 立即刷新状态文本 */
}

/* 文本控件通知（键盘输入回传，预留） */
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  {
      GPIO_InitTypeDef gpio_init = {0};
      gpio_init.Pin  = GPIO_PIN_4;
      gpio_init.Mode = GPIO_MODE_IT_FALLING;
      gpio_init.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(GPIOE, &gpio_init);
      HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }

  /* ��ʼ��Ӳ������ */
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

  /* ===== 串口屏 HMI 初始化（USART1 = RXD0/TXD0） ===== */
  queue_reset();
  UartInit(115200);
  HAL_Delay(300);   /* 等待串口屏上电初始化完毕 */
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* ===== 串口屏触摸指令处理 ===== */
    HMI_Process();

    /* KEY0��ѭ���л�ģʽ��4��ģʽ�� */
    if(key0_pressed)
    {
        key0_pressed = 0;
        HAL_Delay(20);
        if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            current_mode = (current_mode + 1) % 4;
            while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET);
        }
    }

    /* KEY1���޸ĵ�ǰģʽ��ֵ */
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

    /* KEY2����Ե�ǰѡ��ͨ������ AUTOSCALE */
    {
        static uint8_t key2_last = 1;
        uint8_t key2_now = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);

        if(key2_last == 1 && key2_now == 0)
        {
            HAL_Delay(20);
            if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
            {
                AutoScaleAll();
                while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET);
            }
        }
        key2_last = key2_now;
    }

    /* ����ˢ�� */
    if(dma_ready)
    {
        dma_ready = 0;

        float g_cur1 = gain_cal[0][v_div_idx1];
        float g_cur2 = gain_cal[1][v_div_idx2];

        for(int i = 0; i < ADC_BUF_SIZE; i++)
        {
            ch1_v[i] = ((adc_dma_buf[i] & 0xFFFF) / 4095.0f * ADC_VREF - ADC_BIAS_V) / g_cur1;
            ch2_v[i] = ((adc_dma_buf[i] >> 16)     / 4095.0f * ADC_VREF - ADC_BIAS_V) / g_cur2;
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

        /* ====== �����ָ��Ļ���ƽ���˲� ====== */
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
        /* 前端偏置已在 ADC 换算时扣除，这里只需保证峰峰值非负 */
        if (v1_avg < 0.0f) v1_avg = 0.0f;
        if (v2_avg < 0.0f) v2_avg = 0.0f;

        int show_len = (int)(8.0f * t_div_table[t_div_idx] * SAMPLE_RATE);

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
        }

        const char *wave_type = IdentifyWave(ch1_v, max1, min1);

        /* ===== 推送到串口屏：波形曲线 + 测量参数 ===== */
        HMI_UpdateWave(trig, show_len, a1_avg, a2_avg, v1_avg, v2_avg);
        HMI_UpdateText(f1_avg, v1_avg, a1_avg, f2_avg, v2_avg, a2_avg, last_phase, wave_type);
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
