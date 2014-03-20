/********************************************************************************
* Name: blinkled.c								*
* Created By: Siddharth Bharat Purohit						*
* PRU C Compiler Version: 2.0.0B1						*
* Description: This example is presented to illustrate the usage of Starterwar	* 
* library originaly developed for main processor can be utilised for accessing 	*
* main core's peripheral by PRU.						*
*********************************************************************************/
#include <gpio_v2.h>
#include <soc_AM335x.h>
#define PRU0_ARM_INTERRUPT 	19

#define GPIO1 			(*(volatile unsigned long *)(0x4804c000))		// The address of the GPIO1 
#define GPIO_INSTANCE_ADDRESS           (SOC_GPIO_1_REGS)
#define SYSCFG (*(&C4+0x01))
int C4 __attribute__((cregister("MEM",near),peripheral));	//only compatible with v1.1.0B1 +
								//add following lines to MEMORY{} in lnk.cmd
								//PAGE 2:
								//	MEM : o = 0x00026000 l = 0x00002000 CREGISTER=4
								
volatile register unsigned int __R31;

void main()
{
	/*Intialise OCP Master port for accessing external memories*/
	SYSCFG&=0xFFFFFFEF;
	/*Start Main Code*/
	int i;
	while(1) {						//remove if an infinite loop is not needed
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 21, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 22, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 23, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 24, GPIO_PIN_LOW);

	for(i=0;i<0x00a00000;i++);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 21, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 22, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 23, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 24, GPIO_PIN_HIGH);

	for(i=0;i<0x00a00000;i++);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 21, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 22, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 23, GPIO_PIN_LOW);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 24, GPIO_PIN_LOW);
	
	for(i=0;i<0x00a00000;i++);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 21, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 22, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 23, GPIO_PIN_HIGH);
	GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 24, GPIO_PIN_HIGH);	
	for(i=0;i<0x00a00000;i++);
	}
	/*Exiting procedure*/
	__R31 = 35;			// Send notification to Host for program completion
	__halt();			//only compatible with v2.0.0B1 + for lower verions of Compiler use
					//asm(" HALT");
}
