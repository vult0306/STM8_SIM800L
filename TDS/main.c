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
#define LED_GPIO_PORT  (GPIOC)              //port use to turn on/off module SIM
#define LED_GPIO_PINS  (GPIO_PIN_3)         //pin use to turn on/off module SIM
#define TIM4_PERIOD       124               //1ms
#define UART_BUFFER 255                     //255 charactor for UART buffer
#define TDS_LIMIT 500                       //watermark value for TDS
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define VREF 3.0                            // analog reference voltage(Volt) of the ADC
#define SCOUNT  30                          // sum of sample point

// #define CMD_AT 0                            //AT command to check status of module SIM
#define CMD_TEXT_MODE 1                     //set text mode to module SIM
#define CMD_READ_SMS 2                      //read SMS command index
#define CMD_SEND_SMS 3                      //send SMS command index
#define CMD_DELE_MODE 4                      //delete SMS command index

// #define LEN_AT 2                            //AT command lenght
#define LEN_TEXT_MODE 9                     //length of text mode command
#define LEN_READ_SMS 9                      //length of read sms command
#define LEN_SEND_SMS 64                     //length of maximum sms command
#define LEN_DELE_SMS 9                      //length of delete sms command
#define LEN_SDT_CMD 22                      //lenght of string contain phone number
#define LEN_RESPOND_OK 6                    //lenght of respond from module SIM

#define MAX_CONTACT 10                      //Maximmum contact stored
#define MAX_SMS MAX_CONTACT                 //Maximum sms stored
#define LEN_ACTIVATE_CODE 5                 //length of activate code
#define TDS_MEASURE_REPEAT 10               //repeat measuring TDS n times to make sure the water is really dirty

#define MAX_WARNING 2                       //number of warning code
#define TDS_OVER_RANGE 0                    //tds over safe range
#define TDS_UNDER_RANGE 1                   //tds probe is probably not plugged into water
//SDT[LEN_SDT_CMD]="AT+CMGS=\"+xxxxxxxxxxx\"";
struct PHONEBOOK {
  char SDT[LEN_SDT_CMD];                 //contact number including send sms command
  bool sms_sent;                       //indicate if message is already sent to this contact
  bool empty;                               //this is an empty contact
};
__IO uint32_t TimingDelay = 0;
static char respond_ok[LEN_RESPOND_OK]={'\n','\r','K','O','\n','\r'};   //respond from module SIM, expected to be "OK"
// volatile char AT[LEN_AT]="AT";                                                   //AT command to check status of module SIM
volatile char AT_TEXT_MODE[LEN_TEXT_MODE]="AT+CMGF=1";                           //set text mode to module SIM
volatile char AT_READ_SMS[LEN_READ_SMS]="AT+CMGR=x";                             //read sms command, x = sms index
volatile char AT_DELE_SMS[LEN_DELE_SMS]="AT+CMGD=x";                             //delete sms command, x = sms index
static char activated_code[LEN_ACTIVATE_CODE]="water";                         //activate code
static char AT_SEND_SMS[MAX_WARNING][LEN_SEND_SMS]={"Quy khach vui long thay loi loc", //warning message to send to customer
                                                    "Dau do khong tiep xuc voi nuoc"}; //warning message to send to customer
char UART_RX[UART_BUFFER];                                              //UART received buffer
volatile uint8_t  RX_count=0, // UART received buffer count
                  wrn_idx=0,        //warning index
                  tds_over_range=0, //count the time tds value get over the limitation
                  tds_under_range=0, //tds value = 0 means the tds probe is not plugged into water
                  sms_wrk=0;    // current working message
struct PHONEBOOK contact[MAX_CONTACT];
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
static void CLK_Config(void);                     //config cpu clock
static void UART1_Config(void);                   //config uC with SIM communication
static void TIM4_Config(void);                    //config timer
static void GPIO_Config(void);                    //config GPIO
static void ADC_Config(void);                     //config ADC to measure TDS
static int getMedianNum(int*);                    //median value of TDS
static void readTDS(void);                        //read TDS value from TDS sensor
static void cmd_send(uint8_t);                    //send command to module SIM
static void eol(void);                            //send break charactor <LF>
static bool check_respond(void);                  //check respond from module SIM after each command
static bool strcmp(char*,char*,uint8_t);          //compare two string/array
static void strcpy(char*,char*,uint8_t);          //copy string 2 to string 1
static void update_phonebook(void);               //add/remove contact
static void inform_customer(void);                //inform customer if TDS value over range or tds probe is not plugged into water
struct PHONEBOOK get_phone_num(void);
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
    uint8_t i;
    //booting system
    CLK_Config();   /* Clock configuration  */
    GPIO_Config();  /* GPIO configuration   */
    ADC_Config();   /* ADC configuration    */
    TIM4_Config();  /* TIM4 configuration   */
    UART1_Config(); /* UART1 configuration  */

    for(i=0;i<MAX_CONTACT;i++){
        strcpy(contact[i].SDT,"AT+CMGS=\"+xxxxxxxxxxx\"",LEN_SDT_CMD);
        contact[i].sms_sent=FALSE;
        contact[i].empty=TRUE;
    }
    cmd_send(CMD_TEXT_MODE);
    delay(2000);
    while (1)
    {
        // Read SMS every 10s to update phonebook
        delay(1000);
        update_phonebook();
        // Measure TDS value every 1s
        // if (TDS value > limit) or (TDS probe is not put into water)
        // send wanning message to all contact in phonebook
        // tds_over_range=0;
        // tds_under_range=0;
        // for(i=0;i<TDS_MEASURE_REPEAT;i++){
        //     delay(1000);
        //     readTDS();
        //     if(tdsValue>TDS_LIMIT){
        //         tds_over_range++;
        //     }
        //     else if(tdsValue == 0){
        //         tds_under_range++;
        //     }
        // }
        // if((tds_over_range == TDS_MEASURE_REPEAT) || (tds_under_range==TDS_MEASURE_REPEAT))
        //     inform_customer();
    }
}

/*-----------------------------------------------------------------*/
/*
 * this function send out warning message to customer if the probe
 * is not plugged into water or the TDS value is bigger the limittation
 */
static void inform_customer(void){
  uint8_t i;
    for(i=0;i<MAX_CONTACT;i++){
        sms_wrk = i+1;
        //not send sms to this contact yet, so send
        if((!contact[i].empty)&&(!contact[i].sms_sent)){
            if(tds_over_range){
                wrn_idx=TDS_OVER_RANGE;
            }
            else if(tds_under_range){
                wrn_idx=TDS_UNDER_RANGE;
            }
            cmd_send(CMD_SEND_SMS);
        }
    }
}

/*-----------------------------------------------------------------*/
/*
 * this function loop through all sms in memory. 
 * If message contain valid activate code:
 *    - If this contact is already in the phonebook -> reset this
 *      contact sending flag. It means we have not sent the warning
 *      message to this contact yet.
 *    - If this is a new contact -> updated to phonebook
 * If message does not contain activate code -> remove this message
 */
static void update_phonebook(void){
    struct PHONEBOOK temp_contact={"AT+CMGS=\"+xxxxxxxxxxx\"",FALSE,TRUE};//no phonenumber, not sent message yet, not in phonebook
    uint8_t i,j,k;
    //go through all sms(read/unread) in sim
    for(i=0;i<MAX_SMS;i++){
        sms_wrk = i+1;
        //AT_READ_SMS[LEN_READ_SMS]="AT+CMGR=x";
        AT_READ_SMS[LEN_READ_SMS-1]=sms_wrk+0x30;//convert to char, sms idx start from 1
        cmd_send(CMD_READ_SMS);
        //check if message contain activate code
        if(RX_count >= 0x0F){//this message in not empty
            //find sms data position
            for(j=RX_count;j>0;j--)
                if(UART_RX[j]=='"')
                    break;
            //compare with activate code
            for(k=0;k<LEN_ACTIVATE_CODE;k++)
                if(activated_code[k]!=UART_RX[j+k+3])
                    break;
            //this message contain valid code
            if(k==LEN_ACTIVATE_CODE){
                //check if this contact is already in the PHONEBOOK
                temp_contact=get_phone_num();
                //compare with each contact in PHONEBOOK
                for(j=0;j<MAX_CONTACT;j++)
                    if((!strcmp(temp_contact.SDT,contact[j].SDT,LEN_SDT_CMD)) || (contact[j].empty))
                        break;
                //There is space in PHONEBOOK, insert this contact
                if(j!=MAX_CONTACT){
                    temp_contact.empty=FALSE;
                    temp_contact.sms_sent=FALSE;
                    contact[j]=temp_contact;
                }
                //else we just ignore this message, the phonebook is full
            }
            //this is a trash message, remove it
            else{
                AT_DELE_SMS[LEN_DELE_SMS-1]=sms_wrk+0x30;//convert to char, sms idx start from 1
                cmd_send(CMD_DELE_MODE);
            }
        }
    }
}


/*-----------------------------------------------------------------*/
/* compare two string/array
 * s1: address of string/array 1
 * s2: address of string/array 2
 * len: number of character
 */
bool strcmp(char*s1,char*s2,uint8_t len){
    bool flag = TRUE;
    while(len>0){
        len-=1;
        if( *(s1+len) != *(s2+len) ){
            flag = FALSE;
            break;
        }
    }
    return flag;
}

/*-----------------------------------------------------------------*/
/* copy two string/array
 * s1: address of destination string/array
 * s2: address of source string/array
 * len: number of character
 */
static void strcpy(char*s1,char*s2,uint8_t len){
  while(len>0){
    *(s1+len-1) = *(s2+len-1);
    len-=1;
  }
}

/*-----------------------------------------------------------------*/
/* check OK respond from module sim
 * <CR><LF>OK<CR><LF>
 */
static bool check_respond(void){
  uint8_t i;
  for(i=UART_BUFFER-1;i>0;i--)
    if(UART_RX[i]=='\n'){
      if(!strcmp(&UART_RX[i],respond_ok,LEN_RESPOND_OK))
        return TRUE;
      else
        return FALSE;
    }
  if(i==0)
    return FALSE;
}

/*-----------------------------------------------------------------*/
/* extract phone number from a sms message
 */
struct PHONEBOOK get_phone_num(void){
    uint8_t i,j;
    struct PHONEBOOK temp={"AT+CMGS=\"+xxxxxxxxxxx\"",FALSE,TRUE};
    //get phone number
    for(i=0;i<RX_count;i++)
        if(UART_RX[i]==',')
            break;
    for(j=0;j<LEN_SDT_CMD-9;j++)
        temp.SDT[9+j]=UART_RX[i+j+2];
    return temp;
}

/*-----------------------------------------------------------------*/
/* check OK respond from module sim
 * <CR><LF>OK<CR><LF>
 */
static void cmd_send(uint8_t cmd_idx){
    int i;
    for(RX_count=UART_BUFFER-1;RX_count>0;RX_count--)
        UART_RX[RX_count]=0;
    switch (cmd_idx){
        // case CMD_AT:
        //     for(i=0;i<LEN_AT;i++)
        //         putchar(*(AT+i));
        //     eol();
        // break;

        case CMD_TEXT_MODE:
            for(i=0;i<LEN_TEXT_MODE;i++)
                putchar(*(AT_TEXT_MODE+i));
            eol();
        break;

        case CMD_READ_SMS:
            for(i=0;i<LEN_READ_SMS;i++)
                putchar(*(AT_READ_SMS+i));
            eol();
        break;

        case CMD_DELE_MODE:
            for(i=0;i<LEN_DELE_SMS;i++)
                putchar(*(AT_DELE_SMS+i));
            eol();
            delay(3000);
        break;

        case CMD_SEND_SMS:
            //send phone number first
            for(i=0;i<LEN_SDT_CMD;i++)
            putchar(*(contact[sms_wrk-1].SDT+i));
            eol();
            delay(100);
            //send message content
            for(i=0;i<LEN_SEND_SMS;i++)
                putchar(*(AT_SEND_SMS[wrn_idx]+i));
                putchar(26);//send break symbol(26=CTR-Z)
        break;
    }
    delay(2000);
    //check respond, expected to be "OK"
    if(check_respond()==TRUE){
      switch (cmd_idx)
      {
        case CMD_SEND_SMS:
          contact[sms_wrk-1].sms_sent = TRUE;
          break;

        default:
          break;
      }
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
  UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
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
  // UART1_SendData8(CR);//\cr
  // /* Loop until the end of transmission */
  // while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  /* Write a character to the UART1 */
  UART1_SendData8('\n');//\lf
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
  /* Loop until the Read data register flag is SET */
  // while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
  return (c);
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
