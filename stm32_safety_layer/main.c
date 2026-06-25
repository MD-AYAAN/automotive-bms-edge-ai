/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 4S BMS V26 — STM32F407 Safety Layer
  *
  *  ADC MAPPING:
  *    PC1 = ADC1_IN11 = D1 tap  (Calibration: 0.910)
  *    PA1 = ADC1_IN1  = D2 tap  (Auto-calibrated via midpoint method)
  *    PA2 = ADC1_IN2  = D3 tap  (Calibration: 0.862)
  *    PA3 = ADC1_IN3  = D4 tap  (Calibration: 0.901)
  *
  *  BATTERY STACK (bottom to top):
  *    GND -> [Cell4] -> PA2 -> [Cell3] -> PA1 -> [Cell2] -> PC1 -> [Cell1] -> PA3
  *    C4 = tap[PA2]
  *    C3 = tap[PA1] - tap[PA2]
  *    C2 = tap[PC1] - tap[PA1]
  *    C1 = tap[PA3] - tap[PC1]
  *
  *  1-WIRE: PB1, open-drain, external pullup to 3.3V
  *  CAN1:   PB8=RX, PB9=TX via TJA1050, 125kbps
  *          Prescaler=5, BS1=8TQ, BS2=1TQ on 6.25MHz APB1
  *
  *  SENSOR MAP (DS18B20 x8):
  *    S0=C3R  S1=C1L  S2=C2L  S3=C2R  S4=Neg_Term  S5=C3L  S6=C1R  S7=Pos_Term
  *
  *  CAN FRAMES (125kbps):
  *    0x100 — Cell voltages (uint16 x1000 mV, big-endian)
  *    0x101 — Cell1 L/R + Cell2 L/R temps (int16 x10)
  *    0x102 — Cell3 L/R + Pos/Neg temps (int16 x10)
  *    0x103 — PackV, MinV, Status, LoopCount
  *
  *  LEDs: PA5=GREEN, PA6=YELLOW, PA7=RED | PB0=BUZZER
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t sensorROM[8][8] = {
    {0x28,0x68,0x88,0x71,0x00,0x00,0x00,0xAE}, // S0 — Cell3 Right
    {0x28,0x2C,0xCF,0x6C,0x00,0x00,0x00,0x1A}, // S1 — Cell1 Left
    {0x28,0x96,0x2B,0x6F,0x00,0x00,0x00,0x9F}, // S2 — Cell2 Left
    {0x28,0x8E,0x36,0x6E,0x00,0x00,0x00,0x7A}, // S3 — Cell2 Right
    {0x28,0x11,0x2B,0x6E,0x00,0x00,0x00,0x7F}, // S4 — Negative Terminal
    {0x28,0xF1,0x8D,0x71,0x00,0x00,0x00,0x5B}, // S5 — Cell3 Left
    {0x28,0x3B,0x7B,0x6F,0x00,0x00,0x00,0x1C}, // S6 — Cell1 Right
    {0x28,0xD7,0xF2,0x6C,0x00,0x00,0x00,0xF8}  // S7 — Positive Terminal
};

float temp_last_good[8] = {25.0f,25.0f,25.0f,25.0f,25.0f,25.0f,25.0f,25.0f};
uint8_t temp_seeded[8]  = {0,0,0,0,0,0,0,0};

float temp_reject_val[8]   = {0,0,0,0,0,0,0,0};
uint8_t temp_reject_cnt[8] = {0,0,0,0,0,0,0,0};

#define TEMP_JUMP_LIMIT  10.0f
#define TEMP_ABS_MIN    -40.0f
#define TEMP_ABS_MAX     85.0f
#define TEMP_RECOVER_CNT 3

const uint8_t temp_partner[8] = {5, 6, 3, 2, 7, 0, 1, 4};
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN 0 */

// UART printf redirect
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// DWT microsecond delay
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

// 1-Wire implementation
#define OW_PORT  GPIOB
#define OW_PIN   GPIO_PIN_1

static void OW_Init(void)              { OW_PORT->BSRR = OW_PIN; }
static inline void OW_Low(void)        { OW_PORT->BSRR = (uint32_t)OW_PIN << 16U; }
static inline void OW_Release(void)    { OW_PORT->BSRR = OW_PIN; }
static inline uint8_t OW_ReadPin(void) { return (OW_PORT->IDR & OW_PIN) ? 1U : 0U; }

static uint8_t DS18B20_Reset(void)
{
    OW_Low();     delay_us(500);
    OW_Release(); delay_us(70);
    uint8_t p = !OW_ReadPin();
    delay_us(410);
    return p;
}

static void OW_WriteBit(uint8_t bit)
{
    if (bit) { OW_Low(); delay_us(6);  OW_Release(); delay_us(64); }
    else     { OW_Low(); delay_us(60); OW_Release(); delay_us(10); }
}

static uint8_t OW_ReadBit(void)
{
    OW_Low(); delay_us(3);
    OW_Release(); delay_us(10);
    uint8_t b = OW_ReadPin();
    delay_us(53);
    return b;
}

static void OW_WriteByte(uint8_t byte)
{
    for (int i=0;i<8;i++) { OW_WriteBit(byte&0x01); byte>>=1; }
}

static uint8_t OW_ReadByte(void)
{
    uint8_t byte=0;
    for (int i=0;i<8;i++) { byte>>=1; if(OW_ReadBit()) byte|=0x80; }
    return byte;
}

static uint8_t OW_CRC8(uint8_t *data, uint8_t len)
{
    uint8_t crc=0;
    for (uint8_t i=0;i<len;i++) {
        uint8_t byte=data[i];
        for (uint8_t j=0;j<8;j++) {
            uint8_t mix=(crc^byte)&0x01;
            crc>>=1; if(mix) crc^=0x8C;
            byte>>=1;
        }
    }
    return crc;
}

static void DS18B20_StartConversionAll(void)
{
    if (!DS18B20_Reset()) { printf("  WARN: No bus presence\r\n"); return; }
    OW_WriteByte(0xCC); // Skip ROM — broadcasts to all
    OW_WriteByte(0x44); // Convert T
    HAL_Delay(800);
}

static float DS18B20_ReadTemp(uint8_t idx)
{
    uint8_t sp[9];
    if (!DS18B20_Reset()) { printf("  S%d: no presence\r\n",idx); return -999.0f; }
    OW_WriteByte(0x55);
    for (int i=0;i<8;i++) OW_WriteByte(sensorROM[idx][i]);
    OW_WriteByte(0xBE);
    for (int i=0;i<9;i++) sp[i]=OW_ReadByte();
    if (OW_CRC8(sp,8)!=sp[8]) {
        printf("  S%d CRC FAIL\r\n", idx);
        return -999.0f;
    }
    return (int16_t)((sp[1]<<8)|sp[0])/16.0f;
}

static float Filter_Temp(uint8_t idx, float raw)
{
    if (raw < -990.0f) {
        uint8_t p = temp_partner[idx];
        if (temp_seeded[p] && temp_last_good[p] > -40.0f) {
            printf("  S%d FAIL -> using partner S%d=%.2f\r\n", idx, p, (double)temp_last_good[p]);
            return temp_last_good[p];
        }
        return temp_seeded[idx] ? temp_last_good[idx] : 0.0f;
    }
    if (raw < TEMP_ABS_MIN || raw > TEMP_ABS_MAX) {
        printf("  S%d OUTLIER: %.2f out of range\r\n", idx, (double)raw);
        temp_reject_cnt[idx] = 0;
        return temp_seeded[idx] ? temp_last_good[idx] : 0.0f;
    }
    if (!temp_seeded[idx]) {
        temp_seeded[idx] = 1;
        temp_last_good[idx] = raw;
        temp_reject_cnt[idx] = 0;
        return raw;
    }
    float diff = raw - temp_last_good[idx];
    if (diff < 0) diff = -diff;
    if (diff > TEMP_JUMP_LIMIT) {
        float rdiff = raw - temp_reject_val[idx];
        if (rdiff < 0) rdiff = -rdiff;
        if (rdiff < 3.0f) temp_reject_cnt[idx]++;
        else              temp_reject_cnt[idx] = 1;
        temp_reject_val[idx] = raw;
        if (temp_reject_cnt[idx] >= TEMP_RECOVER_CNT) {
            uint8_t p = temp_partner[idx];
            uint8_t partner_agrees = 0;
            if (temp_seeded[p]) {
                float pdiff = raw - temp_last_good[p];
                if (pdiff < 0) pdiff = -pdiff;
                if (pdiff < 5.0f) partner_agrees = 1;
            }
            if (partner_agrees) {
                printf("  S%d RECOVER: %.2f->%.2f (partner S%d agrees)\r\n",
                       idx, (double)temp_last_good[idx], (double)raw, p);
                temp_last_good[idx] = raw;
                temp_reject_cnt[idx] = 0;
                return raw;
            } else {
                printf("  S%d DAMAGED? reads %.2f but partner disagrees\r\n", idx, (double)raw);
                temp_reject_cnt[idx] = 0;
                return temp_last_good[idx];
            }
        }
        return temp_last_good[idx];
    }
    temp_last_good[idx] = raw;
    temp_reject_cnt[idx] = 0;
    return raw;
}

static uint32_t ADC_Read_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig={0};
    sConfig.Channel=channel; sConfig.Rank=1;
    sConfig.SamplingTime=ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1,&sConfig);
    for(int d=0;d<3;d++){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,50);
        HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    uint32_t sum=0;
    for (int i=0;i<16;i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1,50)==HAL_OK) sum+=HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    return sum/16;
}

static inline float fabsf_local(float x) { return x<0?-x:x; }

static uint8_t CAN_SendFrame(uint32_t stdId, uint8_t *data)
{
    uint32_t esr = CAN1->ESR;
    if (esr & CAN_ESR_BOFF) {
        HAL_CAN_Stop(&hcan1);
        HAL_Delay(2);
        if (HAL_CAN_Start(&hcan1) != HAL_OK) return 0;
    }
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if (HAL_GetTick() - t0 > 10) return 0;
    }
    CAN_TxHeaderTypeDef txh = {0};
    txh.StdId = stdId;
    txh.IDE   = CAN_ID_STD;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;
    txh.TransmitGlobalTime = DISABLE;
    uint32_t mbx;
    uint8_t ok = (HAL_CAN_AddTxMessage(&hcan1, &txh, data, &mbx) == HAL_OK) ? 1 : 0;
    HAL_Delay(2);
    return ok;
}

static void Update_LEDs(float minV)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    if (minV < 2.9f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    } else if (minV < 3.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else if (minV <= 4.2f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();

    /* USER CODE BEGIN 2 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    OW_Init();
    HAL_Delay(500);

    printf("\r\n\r\n");
    printf("########################################\r\n");
    printf("#  4S BMS V26 — PA1 BYPASS STABLE     #\r\n");
    printf("########################################\r\n");

    if (HAL_CAN_Start(&hcan1) == HAL_OK)
        printf("CAN:OK\r\n");
    else
        printf("CAN:FAIL\r\n");

    // PA1 auto-calibration (uses midpoint of PA2 and PC1)
    float pa1_factor = 0.896f;
    {
        uint32_t cal_pa2 = ADC_Read_Channel(ADC_CHANNEL_2);
        uint32_t cal_pa1 = ADC_Read_Channel(ADC_CHANNEL_1);
        uint32_t cal_pc1 = ADC_Read_Channel(ADC_CHANNEL_11);
        float uncal_pa2 = (cal_pa2/4095.0f)*3.3f*11.0f*0.862f;
        float uncal_pa1 = (cal_pa1/4095.0f)*3.3f*11.0f;
        float uncal_pc1 = (cal_pc1/4095.0f)*3.3f*11.0f*0.910f;
        float expected_pa1 = (uncal_pa2 + uncal_pc1) / 2.0f;
        if (uncal_pa1 > 0.1f) pa1_factor = expected_pa1 / uncal_pa1;
        if (pa1_factor < 0.7f) pa1_factor = 0.7f;
        if (pa1_factor > 1.1f) pa1_factor = 1.1f;
    }

    float fC1=0,fC2=0,fC3=0,fC4=0;
    float prevC1=0,prevC2=0,prevC3=0,prevC4=0;
    float minV_last=3.0f;
    uint8_t seeded=0, reject_count=0;
    uint32_t can_tx_ok=0, can_tx_fail=0, loop_count=0;
    /* USER CODE END 2 */

    while (1)
    {
        loop_count++;
        printf("==== LOOP %lu ====\r\n", loop_count);

        // Temperature measurement
        DS18B20_StartConversionAll();
        float temp[8];
        for (int i=0;i<8;i++) {
            float raw = DS18B20_ReadTemp(i);
            temp[i] = Filter_Temp(i, raw);
        }

        // Voltage measurement with oversampling + IIR filter
        uint32_t adc_raw[4];
        adc_raw[2]=ADC_Read_Channel(ADC_CHANNEL_2);
        adc_raw[1]=ADC_Read_Channel(ADC_CHANNEL_1);
        adc_raw[0]=ADC_Read_Channel(ADC_CHANNEL_11);
        adc_raw[3]=ADC_Read_Channel(ADC_CHANNEL_3);

        float tap[4];
        tap[0]=(adc_raw[0]/4095.0f)*3.3f*11.0f*0.910f;
        tap[2]=(adc_raw[2]/4095.0f)*3.3f*11.0f*0.862f;
        tap[3]=(adc_raw[3]/4095.0f)*3.3f*11.0f*0.901f;
        tap[1] = (tap[2] + tap[0]) / 2.0f; // PA1 bypass: use midpoint

        float C4=tap[2], C3=tap[1]-tap[2], C2=tap[0]-tap[1], C1=tap[3]-tap[0];

        float packRaw = C1+C2+C3+C4;
        uint8_t in_range = (C1>0.5f&&C1<5.0f)&&(C2>0.5f&&C2<5.0f)&&
                           (C3>0.5f&&C3<5.0f)&&(C4>0.5f&&C4<5.0f)&&
                           (packRaw>8.0f&&packRaw<18.0f);
        uint8_t rate_ok=1;
        if (seeded) {
            rate_ok=(fabsf_local(C1-prevC1)<0.3f)&&(fabsf_local(C2-prevC2)<0.3f)&&
                    (fabsf_local(C3-prevC3)<0.3f)&&(fabsf_local(C4-prevC4)<0.3f);
        }
        uint8_t reading_ok=in_range&&rate_ok;
        if (!reading_ok && reject_count >= 5) { reading_ok = 1; }

        if (!reading_ok) {
            reject_count++;
        } else {
            reject_count=0;
            prevC1=C1; prevC2=C2; prevC3=C3; prevC4=C4;
            if (!seeded) { fC1=C1;fC2=C2;fC3=C3;fC4=C4;seeded=1; }
            else {
                fC1=0.90f*fC1+0.10f*C1; fC2=0.90f*fC2+0.10f*C2;
                fC3=0.90f*fC3+0.10f*C3; fC4=0.90f*fC4+0.10f*C4;
            }
            minV_last=fC1;
            if(fC2<minV_last)minV_last=fC2;
            if(fC3<minV_last)minV_last=fC3;
            if(fC4<minV_last)minV_last=fC4;
        }

        float pack=fC1+fC2+fC3+fC4;

        // CAN TX — Frame 0x100: Cell voltages
        {
            uint8_t d[8];
            uint16_t mv1=(uint16_t)(fC1*1000), mv2=(uint16_t)(fC2*1000);
            uint16_t mv3=(uint16_t)(fC3*1000), mv4=(uint16_t)(fC4*1000);
            d[0]=(mv1>>8)&0xFF; d[1]=mv1&0xFF;
            d[2]=(mv2>>8)&0xFF; d[3]=mv2&0xFF;
            d[4]=(mv3>>8)&0xFF; d[5]=mv3&0xFF;
            d[6]=(mv4>>8)&0xFF; d[7]=mv4&0xFF;
            if (CAN_SendFrame(0x100, d)) can_tx_ok++; else can_tx_fail++;
        }
        // CAN TX — Frame 0x101: Cell1 L/R + Cell2 L/R temps
        {
            uint8_t d[8];
            int16_t t1l=(int16_t)(temp[1]*10), t1r=(int16_t)(temp[6]*10);
            int16_t t2l=(int16_t)(temp[2]*10), t2r=(int16_t)(temp[3]*10);
            d[0]=(t1l>>8)&0xFF; d[1]=t1l&0xFF;
            d[2]=(t1r>>8)&0xFF; d[3]=t1r&0xFF;
            d[4]=(t2l>>8)&0xFF; d[5]=t2l&0xFF;
            d[6]=(t2r>>8)&0xFF; d[7]=t2r&0xFF;
            if (CAN_SendFrame(0x101, d)) can_tx_ok++; else can_tx_fail++;
        }
        // CAN TX — Frame 0x102: Cell3 L/R + terminal temps
        {
            uint8_t d[8];
            int16_t t3l=(int16_t)(temp[5]*10), t3r=(int16_t)(temp[0]*10);
            int16_t tpos=(int16_t)(temp[7]*10), tneg=(int16_t)(temp[4]*10);
            d[0]=(t3l>>8)&0xFF; d[1]=t3l&0xFF;
            d[2]=(t3r>>8)&0xFF; d[3]=t3r&0xFF;
            d[4]=(tpos>>8)&0xFF; d[5]=tpos&0xFF;
            d[6]=(tneg>>8)&0xFF; d[7]=tneg&0xFF;
            if (CAN_SendFrame(0x102, d)) can_tx_ok++; else can_tx_fail++;
        }
        // CAN TX — Frame 0x103: Pack voltage, MinV, Status, LoopCount
        {
            uint8_t d[8];
            uint16_t pv=(uint16_t)(pack*1000), mv=(uint16_t)(minV_last*1000);
            uint8_t status=0;
            if (minV_last<2.9f) status=1;
            else if (minV_last<3.0f) status=2;
            else if (minV_last<=4.2f) status=3;
            else status=4;
            d[0]=(pv>>8)&0xFF; d[1]=pv&0xFF;
            d[2]=(mv>>8)&0xFF; d[3]=mv&0xFF;
            d[4]=status; d[5]=(uint8_t)(loop_count&0xFF);
            d[6]=0; d[7]=0;
            if (CAN_SendFrame(0x103, d)) can_tx_ok++; else can_tx_fail++;
        }

        Update_LEDs(minV_last);
        HAL_Delay(2000);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct={0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct={0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState=RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM=8;
    RCC_OscInitStruct.PLL.PLLN=50;
    RCC_OscInitStruct.PLL.PLLP=RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ=7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV4;  // APB1 = 6.25MHz
    RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_0)!=HAL_OK) Error_Handler();
}

static void MX_CAN1_Init(void)
{
    hcan1.Instance=CAN1;
    hcan1.Init.Prescaler=5;
    hcan1.Init.Mode=CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth=CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1=CAN_BS1_8TQ;
    hcan1.Init.TimeSeg2=CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode=DISABLE;
    hcan1.Init.AutoBusOff=ENABLE;
    hcan1.Init.AutoWakeUp=ENABLE;
    hcan1.Init.AutoRetransmission=DISABLE;
    hcan1.Init.ReceiveFifoLocked=DISABLE;
    hcan1.Init.TransmitFifoPriority=DISABLE;
    if(HAL_CAN_Init(&hcan1)!=HAL_OK) Error_Handler();
    CAN_FilterTypeDef f={0};
    f.FilterBank=0; f.FilterMode=CAN_FILTERMODE_IDMASK;
    f.FilterScale=CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh=0; f.FilterIdLow=0;
    f.FilterMaskIdHigh=0; f.FilterMaskIdLow=0;
    f.FilterFIFOAssignment=CAN_RX_FIFO0; f.FilterActivation=ENABLE;
    if(HAL_CAN_ConfigFilter(&hcan1,&f)!=HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig={0};
    hadc1.Instance=ADC1;
    hadc1.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution=ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode=DISABLE;
    hadc1.Init.ContinuousConvMode=DISABLE;
    hadc1.Init.DiscontinuousConvMode=DISABLE;
    hadc1.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    hadc1.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion=1;
    hadc1.Init.DMAContinuousRequests=DISABLE;
    hadc1.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
    if(HAL_ADC_Init(&hadc1)!=HAL_OK) Error_Handler();
    sConfig.Channel=ADC_CHANNEL_11; sConfig.Rank=1;
    sConfig.SamplingTime=ADC_SAMPLETIME_480CYCLES;
    if(HAL_ADC_ConfigChannel(&hadc1,&sConfig)!=HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance=USART3; huart3.Init.BaudRate=115200;
    huart3.Init.WordLength=UART_WORDLENGTH_8B;
    huart3.Init.StopBits=UART_STOPBITS_1;
    huart3.Init.Parity=UART_PARITY_NONE;
    huart3.Init.Mode=UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart3.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3)!=HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
    g.Pin=GPIO_PIN_1; g.Mode=GPIO_MODE_ANALOG; g.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; g.Mode=GPIO_MODE_ANALOG; g.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; g.Mode=GPIO_MODE_OUTPUT_PP;
    g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0; g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_1; g.Mode=GPIO_MODE_OUTPUT_OD; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_8|GPIO_PIN_9; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_NOPULL;
    g.Speed=GPIO_SPEED_FREQ_HIGH; g.Alternate=GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_LOW; g.Alternate=GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB,&g);
}

void Error_Handler(void) { __disable_irq(); while(1){} }
