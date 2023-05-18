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
// see Chapter 24 in MCU doc
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
  PORTC->PCR[3] |= PORT_PCR_IRQC(10); // Poñer estado do botón para a detección das interrupcións
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 12);
  PORTC->PCR[12] |= PORT_PCR_IRQC(10); // Poñer estado do botón para a detección das interrupcións
}

void enableInterrupt()
{
  // Activar interrupcions dos portos C e D;
  NVIC_SetPriority(PORTC_PORTD_IRQn,0);
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void disableInterrupt()
{
  // Activar interrupcions dos portos C e D;
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
  GPIOE->PSOR = (1 << 29);
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
  GPIOD->PSOR = (1 << 5);
  GPIOE->PTOR = (1 << 29);
}

// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

void enable_TPM()
{
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Darlle reloxo ao TPM
  //MCG->C1 |= MCG_C1_IRCLKEN(0x00);
  //MCG->C2 |= MCG_C2_IRCS(0x00); // Escollemos o modo lento do MCG
  //MCG->SC |= MCG_SC_FCRDIV(0x07); // Divisor da frecuencia do reloxo MCGIRCLK
  //MCG->C1 |= MCG_C1_IRCLKEN(0x01); // Activar fast IRC
  //MCG->S |= MCG_S_IRCST(0x00);
  //MCG->S |= MCG_S_IRCST(0x00); // Escollemos que se use a freq rapida do MCGIRCLK
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0x03); // Escoller o MCGIRCLK como fonte do reloxo, por defecto esta a 32KHz
  TPM0->SC = TPM_SC_CMOD(0x00); // Desactivase para poder editar as opcions
  TPM0->SC |= TPM_SC_CPWMS(0x00); // Escollemos que sexa up-counting
  TPM0->SC |= TPM_SC_PS(0x00); // Factor de division do Prescale, nos non dividimos aqui dado que xa esta a 32KHz
  //TPM0->MOD = TPM_MOD_MOD(0x7A12); // Rexistro do modulo que activa o flag TOF
  TPM0->MOD = TPM_MOD_MOD(0x7CFF); // Numero do counter ao que ten que chegar en cada conta 31999+1
  TPM0->SC |= TPM_SC_TOF_MASK; // Mask para resetear o bit de TOF
  TPM0->SC |= TPM_SC_CMOD(0x01); // Activase de novo, xa estaría funcionando como queremos
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
      segundos = 0;
      minutos++;
      lcd_display_time(minutos, segundos);
      TPM0->SC |= TPM_SC_TOF_MASK;
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

  while(1){
    
  }

  return 0;
}
