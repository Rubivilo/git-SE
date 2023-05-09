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
SemaphoreHandle_t mutex;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
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
	lcd_display_dec(10);
	vTaskDelay(500/portTICK_RATE_MS);
}

void taskConsumidor(){
	lcd_display_dec(20);
	vTaskDelay(200/portTICK_RATE_MS);
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
	//lcd_display_det(1.1);
	irclk_ini();
	lcd_ini();
	lcd_display_dec(0);
	cola = xQueueCreate(50,sizeof(data));
	mutex = xSemaphoreCreateMutex();
	led_green_init();
	led_red_init();

	/* create green led task */
	xTaskCreate(taskProductor, (signed char *)"TaskLedGreen", 
		configMINIMAL_STACK_SIZE, (void *)NULL, prod_prio, NULL);

	/* create red led task */
	xTaskCreate(taskConsumidor, (signed char *)"TaskLedRed", 
		configMINIMAL_STACK_SIZE, (void *)NULL, cons_prio, NULL);
	
	/* start the scheduler */
	vTaskStartScheduler();

	/* should never reach here! */
	for (;;);

	return 0;
}
//SysTick
