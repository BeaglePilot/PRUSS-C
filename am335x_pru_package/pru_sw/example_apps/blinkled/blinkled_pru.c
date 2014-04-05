#define PRU0_ARM_INTERRUPT 	19

#define GPIO1 			(*(volatile unsigned long *)(0x4804c000))		// The address of the GPIO1 
#define GPIO_CLEARDATAOUT 	0x190
#define GPIO_SETDATAOUT 	0x194
#define SYSCFG                  (*(&C4+0x01))
int C4 __attribute__((cregister("MEM",near),peripheral));	//only compatible with v1.1.0B1 +
								//add following lines to MEMORY{} in lnk.cmd
								//PAGE 2:
								//	MEM : o = 0x00026000 l = 0x00002000 CREGISTER=4

volatile register unsigned int __R31;

void main()
{
	/*Intialise OCP Master port for accessing external memories*/
        SYSCFG&=0xFFFFFFEF;       // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    	
	/*Start Main Code*/
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
	__R31 = 35;			// Send notification to Host for program completion
	__halt();			//only compatible with v2.0.0B1 + for lower verions of Compiler use
					//asm(" HALT");
}
