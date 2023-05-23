/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "MKL46Z4.h"
#include "pin_mux.h"
#include <string.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
int verde=0, rojo=0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
 void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
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

// LED_RED = PTE29
void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void turn_green_led_off()
{
	GPIOD->PSOR |= (1 << 5);
	
}

void turn_red_led_off()
{
	GPIOE->PSOR |= (1 << 29);
}

void turn_green_led_on()
{
	GPIOD->PCOR |= (1 << 5);
}

void turn_red_led_on()
{
	GPIOE->PCOR |= (1 << 29);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

void led_red_toggle()
{
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
//comun a ambos botones
void sws_ini(){
  SIM->COPC = 0;             
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  NVIC_SetPriority(31, 0);  
  NVIC_EnableIRQ(31);  
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
  PORTC->PCR[3] |= PORT_PCR_MUX(1); 
  PORTC->PCR[3] |= PORT_PCR_PE_MASK;
  PORTC->PCR[3] |= PORT_PCR_PS_MASK;

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);    
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  PORTC->PCR[12] |= PORT_PCR_MUX(1); 
  PORTC->PCR[12] |= PORT_PCR_PE_MASK;
  PORTC->PCR[12] |= PORT_PCR_PS_MASK;

  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA);   
}

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ

  // SW1
  if(pressed_switch == (0x8)) {
    turn_green_led_on();
    turn_red_led_off();
    verde++;
    rojo=0;
  }
  // SW2
  if(pressed_switch == (0x1000)) {
    turn_red_led_on();
    turn_green_led_off();
    rojo++;
    verde=0;
  }
}
int main(void)
{
  char ch;

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  PRINTF("\r\nReinicio!\r\n");

  while (1)
    {
      ch = GETCHAR();
      PUTCHAR(ch);
    }
}
