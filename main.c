#include <iostm8s003f3.h>
#include <intrinsics.h>
#include "define.h"
#include "eeprom.h"
#include <stdio.h>
#include <stdbool.h>

#define TOTAL_SONGS 5 // Total number of songs
#define DEBOUNCE_DELAY_MS 50 // Debounce delay time in milliseconds

volatile int current_song = 1; // Current song (starting from 1)
volatile bool button_pressed = false;
volatile uint32_t last_button_press_time = 0;
volatile bool tick = false;

void SPI_Init()
{
  SPI_CR1_LSBFIRST = 0;
  SPI_CR1_SPE = 1;
  SPI_CR1_BR = 7; // speed
  SPI_CR1_MSTR = 1;
  SPI_CR1_CPOL = 0;
  SPI_CR1_CPHA = 0;
  //----------------------//
  SPI_CR2_BDM = 1;
  SPI_CR2_BDOE = 1;
  SPI_CR2_CECEN = 0;
  SPI_CR2_CRCNEXT = 0;
  SPI_CR2_RXONLY = 0;
  SPI_CR2_SSM = 0;
  //SPI_CR2_SSI = 1;
}

void HSI_Init(){
  // Configure to use the internal 16MHz oscillator (HSI)
  CLK_ICKR_HSIEN = 1;  // Enable the internal oscillator (HSI)
  CLK_ECKR_HSEEN = 0;  // Disable the external oscillator (HSE)
  // Wait for the internal oscillator (HSI) to stabilize
  while (CLK_ICKR_HSIRDY == 0);
  // Set the divider for HSI (internal oscillator)
  CLK_CKDIVR = 0x00;  // No division (16MHz)
  // Set the main clock source to HSI (internal oscillator)
  CLK_SWCR = 0xE1;    // Set SWEN bit to initiate clock source switch
  CLK_SWCR = 0xE2;    // Set SWIEN and SWEN to select HSI as the main clock source
  // Wait for the switch to complete
  while (CLK_SWCR_SWIF == 0);
}

void Init_Timer2(void)
{
  TIM2_PSCR_PSC = 0; // No prescaling, for a 16MHz count frequency
  TIM2_EGR_UG = 1; // Generate an update event to load the new prescaler
  TIM2_CNTRL = 0x00; // Set this register to 0 for no additional prescaling
  TIM2_CNTRH = 0x00; // Set the initial value for the counter high byte
  TIM2_CNTRL = 0x00; // Set the initial value for the counter low byte
  TIM2_SR1_UIF = 0; // Clear the update interrupt flag
  TIM2_IER_UIE = 1; // Enable update interrupts
  TIM2_CR1_CEN = 1; // Start the timer
  __enable_interrupt(); 
}

void Init_Port(void)
{ 
  PA_DDR = 0x04;
  PA_CR1 = 0x04; // 0: Floating, 1: Pull Up
  PA_CR2 = 0x00; //External interrupt dissable  
  PA_ODR = 0x00;
  
  PB_DDR = 0x30;
  PB_CR1 = 0x30;
  PB_CR2 = 0x30;
  PB_ODR = 0xff;
 
  PC_DDR = 0xff;
  PC_CR1 = 0xff;
  PC_CR2 = 0xff;
  PC_ODR = 0xff;
  
  PD_DDR = 0x64;
  PD_CR1 = 0xff;
  PD_CR2 = 0x64;
  PD_ODR = 0x00;
  
  // Configure GPIO pins as outputs for 7-segment LED control
  PB_DDR |= 0x10; // PB4 is an output
//  PC_DDR |= 0xE0; // PC5, PC6, PC7 are outputs. Because "PC_DDR = 0xff" before so this be commented
  PD_DDR |= 0x0E; // PD1, PD2, PD3 are outputs
  // Configure GPIO pin as an output for the buzzer
  PC_DDR |= 0x08; // PC3 is an output
  // Configure GPIO pin as an input for the button
  PC_DDR &= 0xEF; // PC4 is an input
  //Because have config for all port C before so this be commented
//  PC_CR1 |= 0x10;  // Enable Pull-Up on PC4
//  PC_CR2 &= 0xEF; // Disable button interrupt on PC4
}


void Init_WatchDog(void)
{
  IWDG_KR = 0xcc; //Enable watchdog
  IWDG_KR = 0x55; 
  IWDG_PR = 6; 
  IWDG_RLR = 0xFF;
  IWDG_KR = 0xaa;
}
void Init_ADC(void)
{
  ADC_CR1_ADON = 1;  // enable
  ADC_CR1_CONT = 0;  // continuos
  ADC_CR1_SPSEL = 0; //fadc = master/2;
  ADC_CSR_AWD = 0;   //No analog watchdog event
  ADC_CSR_EOCIE = 0; //EOC interrupt disabled
  ADC_CSR_AWDIE = 0; //AWD interrupt disabled
  ADC_CR2_ALIGN = 1; //right align
}
unsigned int read_adc(unsigned char channel)
{
  unsigned int adc_value;
  unsigned char DATAL;
  unsigned char DATAH;
  ADC_CSR_CH = channel;
  ADC_CR1_ADON = 1;
  while(ADC_CSR_EOC == 0);
  DATAL = ADC_DRL;
  DATAH = ADC_DRH;
  adc_value =(unsigned int)(DATAL|(unsigned int)DATAH<<8);
  return adc_value;
}
unsigned long map(unsigned long x, unsigned long in_min, unsigned long in_max, unsigned long out_min, unsigned long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int main( void )
{ 
  CLK_CKDIVR_HSIDIV = 0;
  Init_Port();
  HSI_Init();
  //SPI_Init();
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  //Number_L = FLASH_ReadByte(0x4000);
  //Number_H = FLASH_ReadByte(0x4001);
 // Number = ((unsigned int)FLASH_ReadByte(0x4000))<<8|FLASH_ReadByte(0x4001);
  DisplayNumber(Number);
  Init_Timer2();
  Init_WatchDog();
  while(1)
  {  
    if(tick)
    {
      // 1ms 
      Vitual_Timer();
      Display7Segment();
      Input_Process();
      Main_Process();     
      
      tick = false;
    }
  }
}

void DelaySec(unsigned int Sec)
{
    if(Sec > 0)
    {
        FlagDelaySec = 1;
        CntDelaySec = Sec;
    }
}
void DelayMiliSec(unsigned int m_sec)
{
    if(m_sec > 0)
    {
        FlagDelay_Msec = 1;
        CntDelay_Msec = m_sec;
    }
}
void OutLed(unsigned char Seg)
{
    if((Seg & 0x1) == 0x1)
        SEG_A = 1;
    else
        SEG_A = 0;
        
    if((Seg & 0x2) == 0x2)
        SEG_B = 1;
    else
        SEG_B = 0;
     
    if((Seg & 0x4) == 0x4)
        SEG_C = 1;
    else
        SEG_C = 0;      
        
    if((Seg & 0x8) == 0x8)
        SEG_D = 1;
    else
        SEG_D = 0; 
        
    if((Seg & 0x10) == 0x10)
        SEG_E = 1;
    else
        SEG_E = 0;  
        
    if((Seg & 0x20) == 0x20)
        SEG_F = 1;
    else
        SEG_F = 0;   
        
    if((Seg & 0x40) == 0x40)
        SEG_G = 1;
    else
        SEG_G = 0; 
}
void Display7Segment()
{              
  switch(IndexOf7Segment)
  {
          case 0: 
                  EN2 = 1;
                  OutLed(Table[Led2]);     
                  EN1 = 0;
                  break;  
          case 1: 
                  EN1 = 1;
                  OutLed(Table[Led1]);  
                  EN2 = 0;
                  break;        
  }
  IndexOf7Segment = (IndexOf7Segment + 1)%2;
}  
void DisplayNumber(unsigned int number)
{
    Led1 = number/10;
    Led2 = number%10;
}

void Main_Process()
{

}
void Input_Process(void)
{
  
  
 if(SW1 == 0 && FlagSW1 == 0)
 {
   FlagSW1 = 1;
   if(Number < 12)
     Number++;
   //if(FlagFirst == 1)DisplayNumber(NumNoteH);
   //else DisplayNumber(Number);
   //OUT1 = !OUT1;
 }
 else if (SW1 == 1 && FlagSW1 == 1)
 {
   FlagSW1 = 0;
 }
 //------------------------------------
 if(SW2 == 0 && FlagSW2 == 0)
 {
   FlagSW2 = 1;
   if(Number > 1)
     Number--;
   //if(FlagFirst == 1)DisplayNumber(NumNoteH);
   //else DisplayNumber(Number);
   //OUT2 = !OUT2;
 }
 else if (SW2 == 1 && FlagSW2 == 1)
 {
   FlagSW2 = 0;
 }
 
 if(FlagRun)DisplayNumber(NumNoteH/2);
 else DisplayNumber(Number);
 
 
 if(INPUT == 0)// Input Opto
 {
    FlagRun = true;
    if(FlagPause == 1)FlagPause = 0;
 }
 else
 { 
   if(FlagEnd == 0)FlagPause = 1;
   else FlagRun = false;
 }

 
 
 if(FlagRun == true)
 {
      //if(FlagFirst == 0)FlagFirst = 1;
      
      //if(FlagFirst ==1)FlagEnd = 0;
     
    //if((FlagPause == 0)&& (FlagEnd == 0))
   if(FlagPause == 0)
    {    
      CntNoteH++;
      if(CntNoteH >= TableTimeH[Number-1][NumNoteH])
      {    
                
          
                    if((NumNoteH%2)==0)
                   {  
                          OUT1 = 1;
                          OUT2 = 1;
                   }
                   else if((NumNoteH%2) == 1)
                   {
                          OUT1 = 0;
                          OUT2 = 0;
                   }
                  
                  NumNoteH ++;
                  if((TableTimeH[Number-1][NumNoteH]==0)||(NumNoteH >= TOTAL_NODE))
                  //if(TableTimeH[Number-1][NumNoteH]==0)
                  {
                        OUT1 = 0;
                        OUT2 = 0;
                        FlagEnd = 1;
                            
                  }
                  else FlagEnd = 0;
                  
                  CntNoteH = 0;
          
      }
    }  
          
 }
 else
 {
    NumNoteH = 0;
    CntNoteH = 0;   
    OUT1 = 0;
    OUT2 = 0;
    //FlagFirst = 0;
 }
 
}
void Vitual_Timer()
{
  if(FlagDelaySec == 1)
    {
        cnt0++;
        if(cnt0 == 1000)
        {
	    cnt0 = 0;
            CntDelaySec--;
            if(CntDelaySec == 0)
            {
                FlagDelaySec = 0;
            }
        }
    }
  if(FlagDelay_Msec == 1)
  {
     CntDelay_Msec--;
     if(CntDelay_Msec == 0)
     FlagDelay_Msec = 0;    
  }  
}
void check_button()
{
    // Read the status of button PC4
    bool button_state = (PC_IDR & (1 << 4)) == 0;

    // Get the current time
    uint32_t current_time = TIM2_CNTRH;
    current_time = (current_time << 16) | TIM2_CNTRL;

    // Check the time between button presses
    if ((current_time - last_button_press_time) >= DEBOUNCE_DELAY_MS)
    {
        if (button_state)
        {
            // Increase the song value
            current_song++;
            // If the song value exceeds the total number of songs, reset to the first song
            if (current_song > TOTAL_SONGS)
            {
                current_song = 1;
            }
            button_pressed = true;
            last_button_press_time = current_time;
        }
    }
}
void change_song()
{
    // Handle song change here, for example:
    // Switch the song based on the value of current_song
    // ...

    // Reset the button state
    button_pressed = false;
}

#pragma vector = TIM2_OVR_UIF_vector
__interrupt void Timer2_ISR(void)
{
  TIM2_CNTRL = 0x2F;
  TIM2_CNTRH = 0xF8;
  tick = true;
  
  IWDG_KR = 0xaa;//clear watchdog
  TIM2_SR1_UIF = 0;
}