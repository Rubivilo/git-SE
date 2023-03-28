#include "MKL46Z4.h"

// LED (RG)
// LED_GREEN = PTD5
// LED_RED = PTE29

void delay(void)
{
  volatile int i;

  for (i = 0; i < 10; i++);
}

// LED_GREEN = PTD5
void led_green_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR |= (1 << 5);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

// LED_RED = PTE29
void led_red_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR |= (1 << 29);
}

void led_red_toggle(void)
{
  GPIOE->PTOR |= (1 << 29);
}


void green_led_off()
{
	GPIOD->PSOR |= (1 << 5);
}

void red_led_off()
{
	GPIOE->PSOR |= (1 << 29);
}

void green_led_on()
{
	GPIOD->PCOR |= (1 << 5);
}

void red_led_on()
{
	GPIOE->PCOR |= (1 << 29);
}

void ptc3_push_button_init()
{
	SIM->COPC = 0;										
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;					
	GPIOC->PDDR &= ~(1 << 3);								
	PORTC->PCR[3] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK);

	PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void ptc12_push_button_init()
{
	SIM->COPC = 0;										
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	GPIOC->PDDR &= ~(1 << 12);
	PORTC->PCR[12] |= (PORT_PCR_MUX(1) | PORT_PCR_PE_MASK);

	PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	}

int is_ptc3_pushed()
{
	return (GPIOC->PDIR & (1 << 3)) == 0;
}

int is_ptc12_pushed()
{
	return (GPIOC->PDIR & (1 << 12)) == 0;
}

void init()
{
	ptc3_push_button_init();
	ptc12_push_button_init();

	led_green_init();
	led_red_init();
}

static int curState = 0;

void PORTDIntHandler(void)
{ 
	int button0 = 0;
	int button1 = 0;
	
	if ((PORTC->ISFR & (1 << 3)) != 0)
		button0 = 1;
	else if ((PORTC->ISFR & (1 << 12)) != 0)
		button1 = 1;

	PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
	PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
	
	switch (curState)
	{
	case 0: // 00
		if (button0)
		{
			if (button1) 
				curState = 3;
			else 
				curState = 2;
		}
		else if (button1)
				curState = 1;
		break;
	case 1: // 01
		if (button0)
		{
			if (button1)
				curState = 2;
			else 
				curState = 3;
		}
		else if (button1)
			curState = 0;
		break;
	case 2: // 10
		if (button0)
		{
			if (button1)
				curState = 1;
			else
				curState = 0;
		}
		else if (button1)
				curState = 3;
		break;
	case 3: // 11
		if (button0)
		{
			if (button1)
				curState = 0;
			else
				curState = 1;
		}
		else if (button1)
				curState = 2;
		break;
	}

	switch(curState)
	{
		case 0:
			green_led_on();
			red_led_off();
			break;
		default:
			green_led_off();
			red_led_on();
	}
}

int main(void)
{
	init();

	while(1);

	return 0;
}