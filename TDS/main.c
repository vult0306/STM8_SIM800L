/**
  ******************************************************************************
  * @file     TIM4_TimeBase\main.c
  * @author   MCD Application Team
  * @version  V2.2.0
  * @date     30-September-2014
  * @brief    This file contains the main function for TIM4 Time Base Configuration example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
/**
  * @addtogroup TIM4_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
/* Private define ------------------------------------------------------------*/
#define LED_GPIO_PORT  (GPIOC)
#define LED_GPIO_PINS  (GPIO_PIN_3)
#define TIM4_PERIOD       124
#define UART_BUFFER 512
#define TDS_LIMIT 500
/* Private macro -------------------------------------------------------------*/
#define convert10to8(x) (uint8_t)(x >> 2)
/* Private variables ---------------------------------------------------------*/
#define VREF 3.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point

#define IDX_AT 0
#define IDX_TEXT_MODE 1
#define IDX_READ_SMS 2
#define IDX_SEND_SMS 3
#define IDX_MAX 4

#define LEN_AT 2
#define LEN_TEXT_MODE 9
#define LEN_READ_SMS 9
#define LEN_SEND_SMS 43
#define LEN_SDT_CMD 20

#define LEN_FB_AT 2
#define LEN_FB_TEXT_MODE 2
#define LEN_FB_READ_SMS 2
#define LEN_FB_SEND_SMS 2

#define MAX_SMS 15
#define CR 0x0d
#define LF 0x0a
#define LEN_ACTIVATE_CODE 5

__IO uint32_t TimingDelay = 0;
char AT[LEN_AT]="AT";
char AT_TEXT_MODE[LEN_TEXT_MODE]="AT+CMGF=1";
char AT_READ_SMS[LEN_READ_SMS]="AT+CMGR=4";
char AT_SDT[LEN_SDT_CMD]="AT+CMGS=+xxxxxxxxxxx";
char activated_code[LEN_ACTIVATE_CODE]="water";
char AT_SEND_SMS[LEN_SEND_SMS]="Nuoc khong du an toan de uong, thay loi loc";
char UART_RX[UART_BUFFER];
bool sms_nsent=TRUE;
bool activated=FALSE;
uint8_t RX_count=0,i=0;
uint16_t Conversion_Value;
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
float compensationCoefficient;
float compensationVolatge;

/* Private function prototypes -----------------------------------------------*/
void delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
static void CLK_Config(void);
static void UART1_Config(void);
static void TIM4_Config(void);
static void GPIO_Config(void);
static void ADC_Config(void);
static void sendsms(void);
static int getMedianNum(int*);
static void readTDS(void);
static void cmd_send(uint8_t);
static void eol (void);
static void get_phone_num(void);
static bool get_customer_info(void);
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    /* Clock configuration -----------------------------------------*/
    CLK_Config();  

    /* GPIO configuration -----------------------------------------*/
    GPIO_Config();  

    /* ADC configuration -------------------------------------------*/
    ADC_Config();

    /* TIM4 configuration -----------------------------------------*/
    TIM4_Config();

    /* UART1 configuration ------------------------------------------------------*/
    UART1_Config();

    while(1){
        delay(1000);
        RX_count=0;
        cmd_send(IDX_AT);
    }
    while (1)
    {
        // delay(10000);
        // readTDS();
        // delay(10000);
        // if tds value exceed the limitation and havent inform customer yet
        // if((tdsValue>TDS_LIMIT) && !warned){
            cmd_send(IDX_TEXT_MODE);
            delay(2000);
            for(i=0;i<MAX_SMS;i++){
                RX_count=0;
                AT_READ_SMS[LEN_READ_SMS-1]=i+0x30+1;//convert to char, sms idx start from 1
                cmd_send(IDX_READ_SMS);
                if(RX_count>0x0F){//message existed
                    if( get_customer_info() ){
                        cmd_send(IDX_SEND_SMS);
                    }
                }
                delay(1000);
            }
        // }
    }
}
static bool get_customer_info(void){
    uint8_t i,j;
    bool temp=TRUE;
    get_phone_num();
    for(i=RX_count;i>0;i--)
        if(UART_RX[i]=='"')
            break;
    for(j=0;j<LEN_ACTIVATE_CODE;j++)
        if(activated_code[j]!=UART_RX[i+j+3])
            temp=FALSE;
    return temp;
}
static void get_phone_num(void){
    uint8_t i,j;
    for(i=0;i<RX_count;i++)
        if(UART_RX[i]==',')
            break;
    for(j=0;j<LEN_SDT_CMD;j++)
        AT_SDT[8+j]=UART_RX[i+j+2];
}

static void cmd_send(uint8_t cmd_idx){
    int i;
    switch (cmd_idx){
        case IDX_AT:
            for(i=0;i<LEN_AT;i++)
                putchar(*(AT+i));
            eol();
        break;
        case IDX_TEXT_MODE:
            for(i=0;i<LEN_TEXT_MODE;i++)
                putchar(*(AT_TEXT_MODE+i));
            eol();
        break;
        case IDX_READ_SMS:
            for(i=0;i<LEN_READ_SMS;i++)
                putchar(*(AT_READ_SMS+i));
            eol();
        break;
        case IDX_SEND_SMS:
            for(i=0;i<LEN_SDT_CMD;i++)
                putchar(*(AT_SDT+i));
            eol();
            delay(100);
            for(i=0;i<LEN_SEND_SMS;i++)
                putchar(*(AT_SEND_SMS+i));
            eol();
        break;
    }
}
/**
  * @brief  Configure system clock to run at 16Mhz
  * @param  None
  * @retval None
  */
static void CLK_Config(void)
{
    /* Initialization of the clock */
    /* Clock divider to HSI/1 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/**
  * @brief  Configure UART1 for the synchronous mode communication
  * @param  None
  * @retval None
  */
static void UART1_Config(void)
{
  /* UART1 configured as follow:
          - Word Length = 8 Bits
          - 1 Stop Bit
          - No parity
          - BaudRate = 9600 baud
          - UART1 Clock enabled
          - Polarity Low
          - Phase Middle
          - Last Bit enabled
          - Receive and transmit enabled
   */
  UART1_DeInit();
  // UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, 
              // (UART1_SyncMode_TypeDef)(UART1_SYNCMODE_CLOCK_ENABLE | UART1_SYNCMODE_CPOL_LOW |UART1_SYNCMODE_CPHA_MIDDLE |UART1_SYNCMODE_LASTBIT_ENABLE),
              // UART1_MODE_TXRX_ENABLE);
  UART1_Init((uint32_t)38400, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  UART1_Cmd(ENABLE);
}

/**
  * @brief  Configure TIM4 to generate an update interrupt each 1ms 
  * @param  None
  * @retval None
  */
static void TIM4_Config(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

/**
  * @brief  Configure GPIO for LEDs available on the evaluation board
  * @param  None
  * @retval None
  */
static void GPIO_Config(void)
{
  /* Initialize I/Os in Output Mode */
  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
}

/**
  * @brief  Configure ADC2 Continuous Conversion with End Of Conversion interrupt 
  *         enabled .
  * @param  None
  * @retval None
  */
static void ADC_Config()
{
  /*  Init GPIO for ADC1 */
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
  
  /* De-Init ADC peripheral*/
  ADC1_DeInit();

  /* Init ADC1 peripheral */
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D2, \
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL3,\
            DISABLE);

  /* Enable EOC interrupt */
  // ADC1_ITConfi-------g(ADC1_IT_EOCIE,ENABLE);

  /* Enable general interrupts */  
  // enableInterrupts();
  
  /*Start Conversion */
  // ADC1_StartConversion();
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while (TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
int getMedianNum(int* bArray) 
{
    int i, j, bTemp;
    int bTab[SCOUNT];
      for (i = 0; i<SCOUNT; i++)
	  bTab[i] = bArray[i];
      for (j = 0; j < SCOUNT - 1; j++) 
      {
	  for (i = 0; i < SCOUNT - j - 1; i++) 
          {
	    if (bTab[i] > bTab[i + 1]) 
            {
		bTemp = bTab[i];
	        bTab[i] = bTab[i + 1];
		bTab[i + 1] = bTemp;
	     }
	  }
      }
      if ((SCOUNT & 1) > 0)
	bTemp = bTab[(SCOUNT - 1) / 2];
      else
	bTemp = (bTab[SCOUNT / 2] + bTab[SCOUNT / 2 - 1]) / 2;
      return bTemp;
}
/**
  * @brief Retargets the C library printf function to the UART.
  * @param c Character to send
  * @retval char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  return (c);
}
static void eol (void)
{
  /* Write a character to the UART1 */
  UART1_SendData8(CR);//\cr
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  /* Write a character to the UART1 */
  UART1_SendData8(LF);//\lf
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  return;
}
/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
    int i=0;
  /* Loop until the Read data register flag is SET */
  // while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
  return (c);
}
void sendsms(){
    cmd_send(IDX_TEXT_MODE);
    delay(100);
    cmd_send(IDX_SEND_SMS);
    delay(100);
}

void readTDS(void){
    /*Start Conversion */
    ADC1_StartConversion();
    while(!(ADC1_GetFlagStatus(ADC1_FLAG_EOC)&0x80));
    analogBuffer[analogBufferIndex] = ADC1_GetConversionValue();
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) 
        analogBufferIndex = 0;

    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    // read the analog value more stable by the median filtering
    // algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp) * (float)VREF / 1024.0;
    //temperature compensation formula:
    //fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    compensationCoefficient=1.0+0.02*(temperature-25.0);
    compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
    //convert voltage value to tds value
    tdsValue=(uint16_t)((133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5);
}
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
