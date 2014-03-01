#define PRU0_ARM_INTERRUPT 	19

#define GPIO1 			(*(volatile unsigned long *)(0x4804c000))		// The address of the GPIO1 
#define GPIO_CLEARDATAOUT 	0x190
#define GPIO_SETDATAOUT 	0x194

void main()
{
	int i;
	(*(volatile unsigned long *)(0x4804c194)) = 7<<22 ;
	for(i=0;i<0x00a00000;i++);
	(*(volatile unsigned long *)(0x4804c190)) = 7<<22;
	for(i=0;i<0x00a00000;i++);
	(*(volatile unsigned long *)(0x4804c194)) = 7<<22 ;
	for(i=0;i<0x00a00000;i++);
	(*(volatile unsigned long *)(0x4804c190)) = 7<<22;
	for(i=0;i<0x00a00000;i++);
	
	/*Exiting procedure*/
	asm(" LDI R3.b0, 35");			// Send notification to Host for program completion
	asm(" HALT");
}
