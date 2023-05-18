#include "MKL46Z4.h"
#include "lcd.h"
#include <stdbool.h> 


// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)

volatile unsigned int minutos=0;
volatile unsigned int segundos=0;
bool settingFlag = false;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3);
  PORTC->PCR[3] |= PORT_PCR_IRQC(10); 
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 12);
  PORTC->PCR[12] |= PORT_PCR_IRQC(10); 
}

void enableInterrupt()
{
  // Activate interruptions on ports C e D;
  NVIC_SetPriority(PORTC_PORTD_IRQn,0);
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void disableInterrupt()
{
  // Deactivate interruptions on ports C e D;
  NVIC_SetPriority(PORTC_PORTD_IRQn,0xFF);
  NVIC_DisableIRQ(PORTC_PORTD_IRQn);
}

int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

// LED_GREEN = PTD5
void led_green_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

// LED_RED = PTE29
void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}
void leds_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
  GPIOD->PTOR = (1 << 5);
}


void enable_TPM()
{
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Darlle reloxo ao TPM
  
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0x03); // MCGIRCLK 
  TPM0->SC = TPM_SC_CMOD(0x00); // TPM counter is disabled
  TPM0->SC |= TPM_SC_CPWMS(0x00); // Up-counting
  TPM0->SC |= TPM_SC_PS(0x00); // Prescale divide by 1
  TPM0->MOD = TPM_MOD_MOD(0x7CFF); // max counter
  TPM0->SC |= TPM_SC_TOF_MASK; // Mask 
  TPM0->SC |= TPM_SC_CMOD(0x01); // TPM enabled the counter increments on every TPM counter clock
}

void check_timer()
{
  if ((TPM0->SC & TPM_SC_TOF_MASK) == TPM_SC_TOF_MASK){
    segundos--;
    if(segundos <= 0 && minutos > 0){
      lcd_display_time(minutos, segundos);
      segundos = 60;
      minutos--;    
    }
    else
      lcd_display_time(minutos, segundos);
    TPM0->SC |= TPM_SC_TOF_MASK;
  }
}

void PORTDIntHandler(void)
{
  // Función que executa o procesador cando recibe unha interrupción do porto C ou D
  PORTC->ISFR = PORT_ISFR_ISF_MASK; // Para desactivar a interrupción

  if(sw1_check()){
    settingFlag=true;
    
  }else if(sw2_check()){
    if(segundos >= 55){
      if(minutos<99){
        segundos = 0;
        minutos++;
        lcd_display_time(minutos, segundos);
        TPM0->SC |= TPM_SC_TOF_MASK;
    }
    }else{
      segundos += 5;
      lcd_display_time(minutos, segundos);
      TPM0->SC |= TPM_SC_TOF_MASK;
    }
  }
}

int main(void)
{
  
  enableInterrupt();

  sw1_ini();
  sw2_ini();
  led_green_ini();
  led_red_ini();
  
  irclk_ini(); // Enable internal ref clk to use by LCD
  enable_TPM();

  lcd_ini();

  lcd_display_time(minutos, segundos);

  while(!settingFlag){

  }

  disableInterrupt();

  while (1) {
    if (minutos<=0 && segundos <= 0){
      break;
    }else
    check_timer();
  }
  LCD->AR =
  LCD_AR_BLINK(1) | //Clear LCD_SEG_AR_BLINK, Disable SLCD blinking. Enable to make LCD Blink
  LCD_AR_BRATE(0x08);
  
  led_red_toggle();
  while(1){
    leds_toggle();
    delay();
  }

  return 0;
}
