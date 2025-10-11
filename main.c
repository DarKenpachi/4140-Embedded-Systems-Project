#include "main.h"

//global var
volatile uint32_t pulse_width = 0;
volatile uint32_t signal_polarity = 0;
volatile uint32_t last_captured = 0;
volatile uint32_t distanceCM = 0;

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//Delays
void Delay(unsigned int n);
void delayMicroseconds(uint32_t microseconds);

//Buzzer
void Init_Buzzer();
void Beep_Detection();

//LEDS
void Init_LED0();

//LCD
void Write_String_LCD(char*);
void Write_Char_LCD(uint8_t);
void Write_Instr_LCD(uint8_t);
void LCD_nibble_write(uint8_t, uint8_t);
void Write_SR_LCD(uint8_t);
void LCD_Init(void);
void writeDistance();

//Key-pad functions
void Keypad_Init();
uint8_t Read_Keypad();

//ULTRASONIC
void Trigger_Init();
void timer_Init();
void TIM4_IRQHandler();
void Trigger_Pulse();
void Echo_Init();



int main(void)
{
	uint8_t temp;
	char* line1;
	line1 = "Term Project2024";
	char* line2;
	line2 = "Distance Meter";
	char* line3;
	line3 = "Press any button";
	char* line4; 
	line4 = "    to Start    ";
	char* line5;
	line5 = "Distance: ";

    //initializations
    HAL_Init();
    SystemClock_Config();
    Trigger_Init();
    Echo_Init();
    timer_Init();
    LCD_Init();
		Keypad_Init();
	
		Write_String_LCD(line1); //print the statement in "line1"
		Write_Instr_LCD(0xc0); // move to line 2
		Write_String_LCD(line2); //print the statement in "line2"

			temp = Read_Keypad();
			if (temp == 14){
				Write_Instr_LCD(0x01); //clear the display//
				Write_String_LCD(line3); //print the statement in "line3"
				Write_Instr_LCD(0xc0); // move to line 2
				Write_String_LCD(line4); //print the statement in "line4"
				}
			if (Read_Keypad() != 0){
				Write_Instr_LCD(0x01); //clear the display//
			}
			
    Write_String_LCD(line5);
    Write_Instr_LCD(0xc0); // move to line 2
    
    while(1){
        Trigger_Pulse(); //send 10 second high to trigger
        writeDistance(); //write to LCD
        Delay(1000); //delay 1 sec
    }
    
}

//#################_SystemClock_Config_########################
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
//##################################################################

//######################_Delay_#############################
void Delay(unsigned int n){
	int i;
		for(;n>0;n--){
			for(i = 0; i < 136; i++){
			}
		}
}
//########################################################


//#################_Microseconds_Delay_########################
void delayMicroseconds(uint32_t microseconds) 
{
    uint32_t start = SysTick->VAL; // Current value of SysTick
    uint32_t ticks = microseconds * (SystemCoreClock / 1000000);
    
    while ((SysTick->VAL - start) < ticks) 
    {
        // Wait for the ticks to pass
    }
}
//####################################################

//#################_LED0_Init_########################
void Init_LED0(){
	uint32_t temp; 
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN;
	temp = GPIOA->MODER;
	temp &=~(0x03<<(2*1));
	temp|=(0x01<<(2*1));
	GPIOA->MODER = temp;
	temp &=~(0x01<<1);
	GPIOA->OTYPER=temp;
	temp=GPIOA->PUPDR;
	temp&=~(0x03<<(2*1));
	GPIOA->PUPDR=temp;
}
//####################################################

//#################_ECHO_Init_########################
void Echo_Init() {
    uint32_t temp;

    // Enable GPIO Port B clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Configure PB6 as alternate function (AF2) for TIM4_CH1
    temp = GPIOB->MODER;
    temp &= ~(0x03 << (2 * 6));     // Clear the current mode for PB6
    temp |= (0x02 << (2 * 6));      // Set PB6 to alternate function mode
    GPIOB->MODER = temp;

    // Set PB6 to the correct alternate function (AF2)
    temp = GPIOB->AFR[0];           // GPIOB->AFR[0] holds AF config for pins 0-7
    temp &= ~(0x0F << (4 * 6));     // Clear current AF setting for PB6
    temp |= (0x02 << (4 * 6));      // Set PB6 to AF2 (TIM4_CH1)
    GPIOB->AFR[0] = temp;

    // No pull-up, pull-down resistors
    temp = GPIOB->PUPDR;
    temp &= ~(0x03 << (2 * 6));     // Set no pull-up/pull-down for PB6
    GPIOB->PUPDR = temp;
}
//####################################################

//#################_TRIGGER_Init_########################
void Trigger_Init(){ //using GPIOB13
    uint32_t temp; 
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOBEN;
	temp = GPIOB->MODER;
	temp &=~(0x03<<(2*13));
	temp|=(0x01<<(2*13));
	GPIOB->MODER = temp;
	temp &=~(0x01<<13);
	GPIOB->OTYPER=temp;
	temp=GPIOB->PUPDR;
	temp&=~(0x03<<(2*13));
	GPIOB->PUPDR=temp;
}
//####################################################

//#################_Timer_Init_########################
void timer_Init(){ //Echo on PB6
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    
    TIM4->PSC = 159;
    TIM4->ARR = 0xFFFF;
    
    TIM4->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
    
    TIM4->CCMR1 &= ~TIM_CCMR1_IC1F;
    
    TIM4->CCER |= TIM_CCER_CC1P|TIM_CCER_CC1NP;
    TIM4->CCMR1 &= ~(TIM_CCMR1_IC1PSC);
    
    TIM4->CCER |= TIM_CCER_CC1E;
    
    TIM4->DIER |= TIM_DIER_CC1IE;
    TIM4->DIER |= TIM_DIER_CC1DE;
    
    TIM4->CR1 |= TIM_CR1_CEN;
    
    NVIC_SetPriority(TIM4_IRQn, 0);
    NVIC_EnableIRQ(TIM4_IRQn);
}
//####################################################

//#################_Timer_Interupt_########################
void TIM4_IRQHandler(){
    
    uint32_t current_captured;
    
    if((TIM4->SR & TIM_SR_CC1IF) != 0){ //is capture input flag set
        current_captured = TIM4->CCR1; //put val of ccr in current_captured
        
        signal_polarity = 1 - signal_polarity; //toggle polarity flag
        
        if(signal_polarity == 0){
            pulse_width = current_captured - last_captured;
        }
     
        last_captured = current_captured;   
        
        TIM4->SR &= ~TIM_SR_CC1IF;
    }
        
    if((TIM4->SR & TIM_SR_UIF) != 0){
        TIM4->SR &= ~(TIM_SR_UIF);
    }    
    
}
//####################################################

//#################_Trigger_Pulse_########################
void Trigger_Pulse(){
    GPIOB->ODR |= (1<<13); //set pin high
    delayMicroseconds(10);
    GPIOB->ODR &= ~(1<<13); //clear pin
}
//####################################################

//#################_LCD_Init_########################
void LCD_Init()
	{
		uint32_t temp;
		
        /* enable GPIOA clock */ 
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 
        /* enable GPIOB clock */ 
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
        /*PB5 MOSI, PA10 /CS_7 latch , PA5 shift clock */

			 
        /*PA5 and PA10 are outputs*/ temp = GPIOA->MODER;
        temp &= ~(0x03<<(2*5)); temp|=(0x01<<(2*5)); 
        temp &= ~(0x03<<(2*10)); temp|=(0x01<<(2*10)); 
        GPIOA->MODER = temp;
			 
        temp=GPIOA->OTYPER;
        temp &=~(0x01<<5);
        temp &=~(0x01<<10); GPIOA->OTYPER=temp;

        temp=GPIOA->PUPDR;
        temp&=~(0x03<<(2*5));
        temp&=~(0x03<<(2*10)); GPIOA->PUPDR=temp;

        /*PB5 is output*/
        temp = GPIOB->MODER;
        temp &= ~(0x03<<(2*5)); 
        temp|=(0x01<<(2*5)); 
        GPIOB->MODER = temp;
        
        temp=GPIOB->OTYPER;
        temp &=~(0x01<<5); 
        GPIOB->OTYPER=temp;
           
        temp=GPIOB->PUPDR;
        temp&=~(0x03<<(2*5)); 
        GPIOB->PUPDR=temp;
        
        /* LCD controller reset sequence */ 
        Delay(20);
        LCD_nibble_write(0x30,0); 
        Delay(5); 
        LCD_nibble_write(0x30,0); 
        Delay(1); 
        LCD_nibble_write(0x30,0);
        Delay(1); 
        LCD_nibble_write(0x20,0); 
        Delay(1);

        Write_Instr_LCD(0x28); /* set 4 bit data LCD - two line display - 5x8 font*/ 
        Write_Instr_LCD(0x0E); /* turn on display, turn on cursor , turn off blinking */ 
        Write_Instr_LCD(0x01); /* clear display screen and return to home position*/ 
        Write_Instr_LCD(0x06); /* move cursor to right (entry mode set instruction)*/
}
//####################################################

//####################_LCD_NIBBLE_####################
void LCD_nibble_write(uint8_t temp, uint8_t s){

/*writing instruction*/ 
if (s==0){ 
	temp=temp&0xF0;
	temp=temp|0x02; /*RS (bit 0) = 0 for Command EN (bit1)=high */ 
	Write_SR_LCD(temp);

    temp=temp&0xFD; /*RS (bit 0) = 0 for Command EN (bit1) = low*/ 
    Write_SR_LCD(temp);	}

/*writing data*/ 
else if (s==1) {
	temp=temp&0xF0;
  temp=temp|0x03;	/*RS(bit 0)=1 for data EN (bit1) = high*/ 
  Write_SR_LCD(temp);

  temp=temp&0xFD; /*RS(bit 0)=1 for data EN(bit1) = low*/ 
  Write_SR_LCD(temp); 
}}
//####################################################

//####################_LCD_String_####################
void Write_String_LCD(char *temp) 
{ 
    int i=0;

    while(temp[i]!=0)
        {
            Write_Char_LCD(temp[i]); i=i+1;
        }
}
//####################################################

//####################_LCD_Instr_####################
void Write_Instr_LCD(uint8_t code)
{
LCD_nibble_write(code&0xF0,0);

code=code<<4; LCD_nibble_write(code,0);
}
//####################################################

//####################_LCD_Char_######################
void Write_Char_LCD(uint8_t code)
{
    LCD_nibble_write(code&0xF0,1);
    code=code<<4;
    LCD_nibble_write(code,1);
}
//####################################################

//####################_LCD_SR_#######################
void Write_SR_LCD(uint8_t temp)
{
int i;
uint8_t mask=0b10000000;
	
for(i=0; i<8; i++) {
        if((temp&mask)==0)
        GPIOB->ODR&=~(1<<5);
        else
        GPIOB->ODR|=(1<<5);

        /*	Sclck */
        GPIOA->ODR&=~(1<<5); GPIOA->ODR|=(1<<5);
        Delay(1);

        mask=mask>>1;
        }
        
    /*Latch*/
    GPIOA->ODR|=(1<<10); 
    GPIOA->ODR&=~(1<<10);
}
//####################################################

//####################_Write_Distance_#######################
void writeDistance(){
    uint32_t pulseTimeUs = (pulse_width * 40);
    uint32_t distanceCM = (pulseTimeUs/58);
    
    uint32_t distanceInt = (uint32_t)distanceCM; //cutting the decimal off
	uint32_t distanceFraction = (distanceCM - distanceInt) * 100;
	
	uint32_t digit1 = distanceInt % 10; //ones place
	uint32_t digit2 = (distanceInt/10)%10; //tens place 
	uint32_t digit3 = (distanceInt/100)%10; //hundreds place
	uint32_t digit4 = distanceFraction / 10;
	uint32_t digit5 = distanceFraction % 10;
    
    Write_Instr_LCD(0xC0); //set cursor to first position on line 2
	Write_Char_LCD(digit3 + 0x30);
	Write_Char_LCD(digit2 + 0x30);
	Write_Char_LCD(digit1 + 0x30);
	Write_Char_LCD('.');
	Write_Char_LCD(digit4 + 0x30);
	Write_Char_LCD(digit5 + 0x30);
	Write_Char_LCD('c');
	Write_Char_LCD('m');
}
//####################################################
void Keypad_Init()
	{
	uint32_t temp;
	/* enable GPIOB clock */ 
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; 

	/*configure input*/
	/* row0 to 3 are PB11, PB10, PB9, PB8 */ 
	temp = GPIOB->MODER;
	temp &= ~(0x03<<(2*11)); 
	temp &= ~(0x03<<(2*10)); temp &= ~(0x03<<(2*9)); 
	temp &= ~(0x03<<(2*8)); 
	GPIOB->MODER = temp;
	temp=GPIOB->OTYPER;
	temp &=~(0x01<<11); 
	temp &=~(0x01<<10); 
	temp &=~(0x01<<9); 
	temp &=~(0x01<<8); 
	GPIOB->OTYPER=temp;

	temp=GPIOB->PUPDR;
	temp&=~(0x03<<(2*11)); 
	temp&=~(0x03<<(2*10)); 
	temp&=~(0x03<<(2*9)); 
	temp&=~(0x03<<(2*8)); 
	GPIOB->PUPDR=temp;

	/* Col 0 to 3 are PB1, PB2, PB3, PB4*/
	/*configure output*/ 
	temp = GPIOB->MODER;
	temp &= ~(0x03<<(2*1)); 
	temp|=(0x01<<(2*1)); 
	temp &= ~(0x03<<(2*2)); 
	temp|=(0x01<<(2*2)); 
	temp &= ~(0x03<<(2*3)); 
	temp|=(0x01<<(2*3));
	temp &= ~(0x03<<(2*4)); 
	temp|=(0x01<<(2*4));
	GPIOB->MODER = temp;

	temp=GPIOB->OTYPER;
	temp &=~(0x01<<1); 
	temp &=~(0x01<<2); 
	temp &=~(0x01<<3); 
	temp &=~(0x01<<4); 
	GPIOB->OTYPER=temp;

	temp=GPIOB->PUPDR;
	temp&=~(0x03<<(2*1));
	temp&=~(0x03<<(2*2)); 
	temp&=~(0x03<<(2*3)); 
	temp&=~(0x03<<(2*4));
	GPIOB->PUPDR=temp;
}
//########################################################################################################


//#################################################################################################
uint8_t Read_Keypad()
{
/* All colomns are zeros col0  PA0  - col1  PA9 - col2 PA10   col3  PB4*/	
	uint8_t a;
	/*set all columns high and wait until a putton is pressed*/ 
	GPIOB->ODR|=(1<<1);
	GPIOB->ODR|=(1<<2); 
	GPIOB->ODR|=(1<<3); 
	GPIOB->ODR|=(1<<4);

	while((GPIOB->IDR &(0x1<<8))==0 && (GPIOB->IDR &(0x1<<9))==0 && (GPIOB->IDR &(0x1<<10))==0 &&	(GPIOB->IDR &(0x1<<11))==0)
	{}
	
	Delay(25);	/*debouncing*/
/*scanning */ 
	while(1){
	GPIOB->ODR&=~(1<<1); 
	GPIOB->ODR&=~(1<<2); 
	GPIOB->ODR&=~(1<<3); 
	GPIOB->ODR&=~(1<<4);
	
	/* Scan Col 0   PB1 = high*/
	/* check rows */
	GPIOB->ODR|=(1<<1);
	Delay(2);
	/* check rows*/
	if((GPIOB->IDR &(0x1<<8))!=0)
		{a=1;
		break;}
	if((GPIOB->IDR &(0x1<<9))!=0) 
		{a=4;
		break;}
	if((GPIOB->IDR &(0x1<<10))!=0)
	 {a=7;
		break;}
	if((GPIOB->IDR &(0x1<<11))!=0)
	{a=14;
		break;}
		
	/* Scan Col 1 */
	GPIOB->ODR&=~(1<<1);
	Delay(2);		
	GPIOB->ODR|=(1<<2);
	Delay(2);	
	if((GPIOB->IDR &(0x1<<8))!=0)
		{a=2;
			break;}
	if((GPIOB->IDR &(0x1<<9))!=0) 
		{a=5;
			break;}
	if((GPIOB->IDR &(0x1<<10))!=0)
		{a=8;
			break;}
	if((GPIOB->IDR &(0x1<<11))!=0)
		{a=0;
			break;}
		
	/* Scan Col 2 */ 
	GPIOB->ODR&=~(1<<2);
	Delay(2);
	GPIOB->ODR|=(1<<3);
	Delay(2);
	if((GPIOB->IDR &(0x1<<8))!=0)
		{a=3;
		break;}
	if((GPIOB->IDR &(0x1<<9))!=0)
		{a=6;
		break;}
	if((GPIOB->IDR &(0x1<<10))!=0)
		{a=9;
		break;}
	if((GPIOB->IDR &(0x1<<11))!=0)
		{a=15;
		break;}

	/* Scan Col 3 */ 
	GPIOB->ODR&=~(1<<3);
	Delay(2);
	GPIOB->ODR|=(1<<4);
	Delay(2);
	if((GPIOB->IDR &(0x1<<8))!=0)
		{a=10;
		break;}
	if((GPIOB->IDR &(0x1<<9))!=0)
		{a=11;
		break;}
	if((GPIOB->IDR &(0x1<<10))!=0)
		{a=12;
		break;}
	if((GPIOB->IDR &(0x1<<11))!=0)
		{a=13;
		break;}

	}
	/*wait until button is released*/ 
	GPIOB->ODR|=(1<<1);
	Delay(2);
	GPIOB->ODR|=(1<<2);
	Delay(2);
	GPIOB->ODR|=(1<<3);
	Delay(2);
	GPIOB->ODR|=(1<<4);
	Delay(2);

	while(!(	((GPIOB->IDR &(0x1<<8))==0) && ((GPIOB->IDR &(0x1<<9))==0) && 	((GPIOB->IDR &(0x1<<10))==0)	&&	((GPIOB->IDR &(0x1<<11))==0) ))
	{}
	Delay(25); 
	return(a);
}
//#######################################################################################

//##################_BUZZER_Init_####################
void Init_Buzzer(){
    uint32_t temp;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    
    temp = GPIOC->MODER;
    temp &= ~(0x03<<(2*9));
    temp |= (0x01<<(2*9));
    GPIOC->MODER = temp;
    
    temp = GPIOC->OTYPER;
    temp &= ~(0x01<<9);
    GPIOC->OTYPER = temp;
    
    temp = GPIOC->PUPDR;
    temp &= ~(0x03 << (2*9));
    GPIOC->PUPDR = temp;
}
//####################################################

//#################_Beep_Detection_###################

//####################################################

//################## Led Detection ####################

//#####################################################


/* USER CODE END 4 */


//#######################System_Code###########################
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
