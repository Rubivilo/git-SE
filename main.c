#include "MKL46Z4.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"

int prod_prio=1;
int cons_prio=2;
int data;
QueueHandle_t cola;
SemaphoreHandle_t qMutex;
SemaphoreHandle_t xMutex;

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



void led_green_init()
{
	SIM_COPC = 0;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD_PCR5 = PORT_PCR_MUX(1);
	GPIOD_PDDR |= (1 << 5);
	GPIOD_PSOR = (1 << 5);
}

void led_green_toggle()
{
	GPIOD_PTOR = (1 << 5);
}

void led_red_init()
{
	SIM_COPC = 0;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE_PCR29 = PORT_PCR_MUX(1);
	GPIOE_PDDR |= (1 << 29);
	GPIOE_PSOR = (1 << 29);
}

void led_red_toggle(void)
{
	GPIOE_PTOR = (1 << 29);
}

void taskProductor(){
	
	if( cola != 0 )
    {
        /* Send an unsigned long.  Wait for 10 ticks for space to become
        available if necessary. */
        if( xQueueSend( cola,
                       2,
                       ( TickType_t ) 10 ) != pdPASS )
        {
            /* Failed to post the message, even after 10 ticks. */
        }
    }
	lcd_display_dec(50-uxQueueSpacesAvailable(cola));

	vTaskDelay(200/portTICK_RATE_MS);
}

void taskConsumidor(){
	if( cola != 0 )
    {
        /* Send an unsigned long.  Wait for 10 ticks for space to become
        available if necessary. */
        if( xQueueRecieve( cola,
                       2,
                       ( TickType_t ) 10 ) != pdPASS )
        {
            /* Failed to post the message, even after 10 ticks. */
        }
    }
	lcd_display_dec(50-uxQueueSpacesAvailable(cola));

	vTaskDelay(500/portTICK_RATE_MS);
}


void taskLedGreen(void *pvParameters)
{
    for (;;) {
        led_green_toggle();
        vTaskDelay(200/portTICK_RATE_MS);
    }
}

void taskLedRed(void *pvParameters)
{
    for (;;) {
        led_red_toggle();
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

int main(void)
{

	irclk_ini();
	lcd_ini();
	lcd_display_dec(0004);
	cola = xQueueCreate(50,sizeof(data));
	xMutex = xSemaphoreCreateMutex();
	qMutex = xSemaphoreCreateMutex();
	led_green_init();
	led_red_init();

	/* create green led task */
	xTaskCreate(taskProductor, (signed char *)"TaskProductor", configMINIMAL_STACK_SIZE, (void *)1, prod_prio, NULL);

	/* create red led task */
	xTaskCreate(taskConsumidor, (signed char *)"TaskConsumidor", configMINIMAL_STACK_SIZE, (void *)1, cons_prio, NULL);
	
	/* start the scheduler */
	vTaskStartScheduler();

	/* should never reach here! */
	for (;;);

	return 0;
}

