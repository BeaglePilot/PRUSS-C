#include <gpio_v2.h>
#include <soc_AM335x.h>
#define PRU0_ARM_INTERRUPT 	19

#define GPIO1 			(*(volatile unsigned long *)(0x4804c000))		// The address of the GPIO1 
#define GPIO_INSTANCE_ADDRESS           (SOC_GPIO_1_REGS)

void main()
{
	/*Intialise OCP Master port for accessing external memories*/
	asm(" LBCO &r0, C4, 4, 4");
    	asm(" CLR r0, r0, 4");         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    	asm(" SBCO &r0, C4, 4, 4");

	/*Start Main Code*/
	int i;
	while(1) {	
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
	asm(" LDI R31.b0, 35");			// Send notification to Host for program completion
	asm(" HALT");
}
