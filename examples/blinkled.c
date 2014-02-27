#define PRU0_ARM_INTERRUPT 	19

#define GPIO1 			(*(volatile unsigned long *)(0x4804c000))		// The adress of the GPIO1 
#define GPIO_CLEARDATAOUT 	0x190
#define GPIO_SETDATAOUT 	0x194

int main()
{
	int i;
	while(1)
	{
		GPIO1=GPIO_CLEARDATAOUT ;
		for(i=0;i<10000;i++);
		GPIO1=GPIO_SETDATAOUT ;
		for(i=0;i<10000;i++);
	}
	return 0;
}
