#include "stm32f4xx.h"
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define GYRO_YOUT_H_REG 0x45
#define GYRO_ZOUT_H_REG 0x47
#define SIZE 500

int hello = 0;
int16_t x_read=0,y_read=0,z_read=0;
float x_accel,y_accel,z_accel;
float x_gyro[SIZE],y_gyro[SIZE],z_gyro[SIZE];
float x_gyro_avg,y_gyro_avg,z_gyro_avg;
uint8_t check,temp;
int test,test1,test2,test3,test4;
int a,b,c,d,e,f,g,h;



//Using Tim3, TIM5, TIM9, TIM10 for the 4 BLDC motors.
void GPIO_Config(void)

{
	RCC->AHB1ENR |=(1UL<<1);//Enable clock for port B for I2C
	GPIOB->MODER |=(2UL<<12);//PB6 to alternate function pin
	GPIOB->MODER |=(2UL<<14);//PB7 to alternate function pin
	GPIOB->PUPDR |=(0x5UL<<12);//set PB6 and 7 as pull up pins
	GPIOB->OTYPER |=(0x3UL<<6);//Set PB6 and 7 as open drain
	GPIOB->OSPEEDR |=(0xAUL<<12);//Set PB6 and 7 as high speed
	GPIOB->AFR[0] |= (0x44<<24);//Set PB6 and 7 to alternate function 4
}

void I2C_Config(){
	RCC ->APB1ENR |= RCC_APB1ENR_I2C1EN; // ENABLE I2C CLOCK
	I2C1 ->CR1 |= I2C_CR1_SWRST; //RESET I2C
	I2C1 ->CR1 &= ~I2C_CR1_SWRST;
	I2C1 ->CR2 |= 16 << 0; // PERIPHERAL CLOCK AT 16MHZ
	I2C1 ->OAR1 |= (1<<14); //HIGH BY SOFTWARE

	// SET CLOCK OF 100KHZ
	// TPCLK = 1/16M. THIGH = 5000NS. THIGH = CCR * TPCLK
	I2C1 ->CCR |= 0x50<<0;
	I2C1 ->TRISE |= 17<<0; // FREQ + 1 (MAX RISE TIME)

	I2C1 ->CR1 |= I2C_CR1_PE; //ENABLE I2C
}

void I2C_Start(){
	I2C1 ->CR1 |= I2C_CR1_ACK; // ENABLE ACK BIT ?
	I2C1 ->CR1 |= I2C_CR1_START; //SET START BIT

	while (!(I2C1->SR1 & I2C_SR1_SB)); // WAIT FOR SB TO SET

}
void I2C_Write(uint8_t data){
	while (!(I2C1 ->SR1 & I2C_SR1_TXE)); // WAIT FOR TX TO GET EMPTY
	I2C1 ->DR = data;
	while (!(I2C1 ->SR1 & I2C_SR1_BTF)); // WAIT FOR BTF TO SET
}
void I2C_Address (uint8_t Address)
{
	I2C1->DR = Address;  //  send the slave address
	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
	temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size){
	int remaining =  size;
	if(size == 1){
		I2C1 ->DR = Address;
		while (!(I2C1->SR1 & I2C_SR1_ADDR));
		I2C1 ->CR1 &= ~I2C_CR1_ACK; // CLEAR ACK
		temp = I2C1->SR1 | I2C1 ->SR2; // CLEAR ADDR.
		I2C1 ->CR1 |= I2C_CR1_STOP; //STOP
		while (!(I2C1->SR1 & I2C_SR1_RXNE)); //WAIT FOR RXNE
		buffer[size - remaining] = I2C1 ->DR;
	}
	else {
		I2C1 ->DR = Address;
		while (!(I2C1->SR1 & I2C_SR1_ADDR));
		temp = I2C1 ->SR1 | I2C1 ->SR2;
		while (remaining > 2){
			while (!(I2C1 ->SR1 & I2C_SR1_RXNE));
			buffer[size - remaining] = I2C1->DR;
			I2C1 ->CR1 |= I2C_CR1_ACK;
			remaining--;
		}
		// READING SECOND LAST BYTE
		while (!(I2C1 ->SR1 & I2C_SR1_RXNE));
		buffer[size - remaining] = I2C1->DR;
		I2C1 ->CR1 &= ~I2C_CR1_ACK;
		I2C1 ->CR1 |= I2C_CR1_STOP;
		remaining--;
		// LAST BYTE
		while (!(I2C1 ->SR1 & I2C_SR1_RXNE));
		buffer[size - remaining] = I2C1->DR;
	}
}
void I2C_Stop (void)
{
	I2C1->CR1 |= (1<<9);  // Stop I2C
}

void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Write (Data);
	I2C_Stop ();
}

void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Start ();
	I2C_Read (Address+0x01, buffer, size);//To read, set LSB to 1
	I2C_Stop ();
}


void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;
	MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);//Check device ID

	if (check == 104)  // 0x68 will be returned by the sensor
	{

		Data = 0;
		MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);//Power up the sensor

		Data = 0x07;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);//Sampling rate of 1KHz

		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);//accelerometer range=+/- 2g

		Data = 0x00;
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG_REG, Data);//accelerometer range=+/- 250 /s


	}
}

void MPU6050_Read_Accel (void)

{

	uint8_t Buf[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Buf, 6);

	x_read = (int16_t)(Buf[0] << 8 | Buf[1]);

	y_read = (int16_t)(Buf[2] << 8 | Buf[3]);

	z_read = (int16_t)(Buf[4] << 8 | Buf[5]);

//	x_accel = (x_read/16384.0) + 0.09;
//
//	y_accel = (y_read/16384.0);
//
//	z_accel = (z_read/16384.0) - 1.07;


	float x_accel_temp = ((x_read/16384.0) + 0.09) * 255;

	float y_accel_temp = (y_read/16384.0) * 255;

	float z_accel_temp = ((z_read/16384.0) - 1.07)*255;

	x_accel = (int) x_accel_temp;
	y_accel = (int) y_accel_temp;
	z_accel = (int) z_accel_temp;

}



void MPU6050_Read_Gyro (void)

{
	int i = SIZE;
	float x=0,y=0,z=0;
//	while(i){

		uint8_t Buf[6];

		// Read 6 BYTES of data starting from GYRO_XOUT_H register

		MPU_Read (MPU6050_ADDR, GYRO_XOUT_H_REG, Buf, 6);

		x_read = (int16_t)(Buf[0] << 8 | Buf[1]);

		y_read = (int16_t)(Buf[2] << 8 | Buf[3]);

		z_read = (int16_t)(Buf[4] << 8 | Buf[5]);

		x_gyro[i-1] = x_read / 131.0; // Sensitivity Scale Factor for FS_SEL = 0 (+- 250 degrees/sec)
		x+=x_gyro[i-1];
		y_gyro[i-1] = y_read / 131.0; // Sensitivity Scale Factor for FS_SEL = 0 (+- 250 degrees/sec)
		y+=y_gyro[i-1];
		z_gyro[i-1] = z_read / 131.0; // Sensitivity Scale Factor for FS_SEL = 0 (+- 250 degrees/sec)
		z+=z_gyro[i-1];
		i--;
//	}
	x_gyro_avg = x + 1.35;
	y_gyro_avg = y - 1;
	z_gyro_avg = z - 2.23;

}
//void GPIO_init(){
//	// Enable clock for GPIOA and Timer 3
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable Timer 3 clock
//
//	// Configure PA6 (TIM3_CH1) and PA7 (TIM3_CH2) as alternate function
//	GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); // Clear mode bits
//	GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Set mode to alternate function (10)
//
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); // Set output type to push-pull (0)
//
//	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7); // Set speed to high
//
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7); // No pull-up, no pull-down (00)
//
//	// Set alternate function to AF2 (TIM3_CH1 and TIM3_CH2) for PA6 and PA7
//	GPIOA->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // Clear alternate function bits
//	GPIOA->AFR[0] |= ((2 << 24) | (2 << 28)); // Set alternate function to AF2 (TIM3_CH1 and TIM3_CH2)
//
//}

void GPIO_init(){
    // Enable clock for GPIOA and GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock

    // Enable clock for Timer 3 and Timer 5 and Timer 10 and Timer 11
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable Timer 3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;  // Enable Timer 5 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; // Enable Timer 10 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; // Enable Timer 11 clock

    // Configure PA6 (TIM3_CH1) as alternate function
    GPIOA->MODER &= ~GPIO_MODER_MODER6; // Clear mode bits for PA6
    GPIOA->MODER |= GPIO_MODER_MODER6_1; // Set mode to alternate function (10)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6; // Set output type to push-pull (0)
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // Set speed to high
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6; // No pull-up, no pull-down (00)
    GPIOA->AFR[0] &= ~(0xF << 24); // Clear alternate function bits for PA6
    GPIOA->AFR[0] |= (2 << 24); // Set alternate function to AF2 (TIM3_CH1)

    // Configure PA0 (TIM5_CH1) as alternate function
    GPIOA->MODER &= ~GPIO_MODER_MODER0; // Clear mode bits for PA0
    GPIOA->MODER |= GPIO_MODER_MODER0_1; // Set mode to alternate function (10)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0; // Set output type to push-pull (0)
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0; // Set speed to high
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0; // No pull-up, no pull-down (00)
    GPIOA->AFR[0] &= ~(0xF << 0); // Clear alternate function bits for PA0
    GPIOA->AFR[0] |= (2 << 0); // Set alternate function to AF2 (TIM5_CH1)


// Configure PB8 (TIM10_CH1) as alternate function
    GPIOB->MODER &= ~GPIO_MODER_MODER10; // Clear mode bits for PB8
    GPIOB->MODER |= GPIO_MODER_MODER10_1; // Set mode to alternate function (10)
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT10; // Set output type to push-pull (0)
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; // Set speed to high
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR10; // No pull-up, no pull-down (00)
    GPIOB->AFR[1] &= ~(0xF << 8); // Clear alternate function bits for PB8
    GPIOB->AFR[1] |= (3 << 8); // Set Alternate Function to AF3 (TIM10_CH1)

//Configure PB9 (TIM11_CH1) as alternate function
     GPIOB->MODER &= ~GPIO_MODER_MODER11; // Clear mode bits for PB8
    GPIOB->MODER |= GPIO_MODER_MODER11_1; // Set mode to alternate function (10)
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT11; // Set output type to push-pull (0)
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11; // Set speed to high
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR11; // No pull-up, no pull-down (00)
    GPIOB->AFR[1] &= ~(0xF << 12); // Clear alternate function bits for PB9
    GPIOB->AFR[1] |= (3 << 12); // Set Alternate Function to AF3 (TIM11_CH1)

}
void TIM3_init(){
	//Pa0 right side, pa6 left
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN; // TIM3 enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN;
	TIM3 ->PSC = 160 - 1;
	TIM3 ->ARR = 2000 - 1; //50hz CREATED FROM 100KHZ
	TIM3 ->CCR1 = 200; //INIT1 CHANNEL

	TIM5 ->PSC = 160 - 1;
	TIM5 ->ARR = 2000 - 1; //50hz CREATED FROM 100KHZ
	TIM5 ->CCR1 = 200; //INIT1 CHANNEL

	TIM10 -> PSC = 160 - 1;
	TIM10 -> ARR = 2000 - 1;
	TIM10 ->CCR1 = 200;

	TIM11 -> PSC = 160 - 1;
	TIM11 -> ARR = 2000 - 1;
	TIM11 ->CCR1 = 200;

//	TIM3 ->CCR2 = 200; // INIT2 CHANNEL
	TIM3 ->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM3->CCER|=1<<0; //ENABLE CAPTURE COMPARE

	TIM5 ->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM5->CCER|=1<<0; //ENABLE CAPTURE COMPARE

	TIM10 ->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM10->CCER|=1<<0; //ENABLE CAPTURE COMPARE

	TIM11 ->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM11->CCER|=1<<0; //ENABLE CAPTURE COMPARE

//	TIM3 ->CCMR2 |= TIM_CCMR1_OC1M_2;
	TIM3 ->SR &= ~TIM_SR_UIF;// AUTO CLEAR IN PWM
	TIM5 ->SR &= ~TIM_SR_UIF;// AUTO CLEAR IN PWM
	TIM10 ->SR &= ~TIM_SR_UIF;// AUTO CLEAR IN PWM
	TIM11 ->SR &= ~TIM_SR_UIF;// AUTO CLEAR IN PWM
}


void delay(){
	RCC ->APB1ENR |= 1<<0;
	TIM2 ->SR &= ~TIM_SR_UIF;
	TIM2 ->PSC = 16000 - 1;
	TIM2 ->ARR = 3000-1;
	TIM2 ->CR1 |= 1<<0;
	while(!(TIM2 ->SR & TIM_SR_UIF));
	TIM2 ->CR1 &= ~(1<<0);
	hello = 1;
}


void BLDC_init(){
	TIM3 ->CR1 |= 1<<0; //START TIMER, 200 INITI
	TIM5 ->CR1 |= 1<<0; //START TIMER, 200 INITI
	TIM10 -> CR1 |= 1<<0;
	TIM11 -> CR1 |= 1<0;
//	while(!(TIM2 ->SR & TIM_SR_UIF));
//	TIM2 ->CR1 &= ~(1<<0);
//	TIM2 ->SR &= ~TIM_SR_UIF;
//	delay();
	TIM5 ->CCR1 = 100;
	TIM3 ->CCR1 = 100;
	TIM10 -> CCR1 = 100;
	TIM11 -> CCR1 = 100;
//	while(!(TIM2 ->SR & TIM_SR_UIF));
//	TIM2 ->SR &= ~TIM_SR_UIF;
	delay();
	TIM3 ->CR1 &= ~(1<<0); //STOP TIMER
	TIM5 ->CR1 &= ~(1<<0); //STOP TIMER
	TIM10 -> CR1 &= ~(1<<0);
	TIM11 -> CR1 &= ~(1<<0);
	TIM5 ->CCR1 = 150;
	TIM3 ->CCR1 = 150;
	TIM10 -> CCR1 = 150;
	TIM11 -> CCR1 = 150;
}

void stabilize_platform_y(){
	MPU6050_Read_Accel ();
	if(y_accel < -30){ //right tilt
		if(TIM5->CCR1 < 200 && TIM3->CCR1 > 100){
			TIM3 ->CCR1 -= 1;
			TIM5 ->CCR1 += 1;
		}
		else if(TIM3->CCR1 < 100){ //CCR2 = 200
			TIM5 ->CCR1 +=1;
		}
		else if(TIM5->CCR1 > 200){ //CCR1 = 100
			TIM3 ->CCR1 -=1;
		}
		else {
			TIM3 ->CCR1 = 150;
			TIM5 ->CCR1 = 150;
		}
	}
	else if(y_accel > 30){ //right tilt
		if(TIM3->CCR1 < 200 && TIM5->CCR1 > 100){
			TIM5 ->CCR1 -= 1;
			TIM3 ->CCR1 += 1;
		}
		else if(TIM5->CCR1 < 100){ //CCR1 = 200
			TIM3 ->CCR1 +=1;
		}
		else if(TIM3->CCR1 > 200){ //CCR2 = 100
			TIM5 ->CCR1 -=1;
		}
		else {
			TIM3 ->CCR1 = 150;
			TIM5 ->CCR1 = 150;
		}
	}
}
void stabilise_platform_x(){
	 if(x_accel > 30){ //Forward Tilt
			if(TIM10 -> CCR1 < 200 && TIM11 -> CCR1 > 100)
			{
				TIM11 -> CCR1 -= 1;
				TIM10 -> CCR1 += 1;
			}
			else if(TIM10 -> CCR1 > 200)
			{
				TIM11 -> CCR1 -= 1;
			}
			else if(TIM11 -> CCR1 < 100)
	{
			TIM10 -> CCR1 += 1;
	}
	else {
			TIM10 -> CCR1 = 150;
			TIM11 -> CCR1 = 150;
			}
	}
	else if( x_accel < -30){ // Backward Tilt
		if(TIM11 -> CCR1 < 200 && TIM10 -> CCR1 > 100)
			{
				TIM10 -> CCR1 -= 1;
				TIM11 -> CCR1 += 1;
			}
			else if(TIM11 -> CCR1 > 200)
			{
				TIM10 -> CCR1 -= 1;
			}
			else if(TIM10 -> CCR1 < 100)
	{
			TIM11 -> CCR1 += 1;
	}
	else {
			TIM11 -> CCR1 = 150;
			TIM10 -> CCR1 = 150;
			}
	}
}




long timer3,timer5, timer10, timer11;


int main ()

{

//	configureLED();
	GPIO_init();
	TIM3_init();
	BLDC_init();
	GPIO_Config();
	I2C_Config ();
	MPU6050_Init ();
	TIM3 ->CR1 |= 1<<0; //START TIMER
	TIM5 ->CR1 |= 1<<0;
	TIM10 -> CR1 |= 1<<0;
	TIM11 -> CR1 |= 1<<0;
	while (1)
	{
//		MPU6050_Read_Accel ();
		stabilize_platform_y();
		stabilise_platform_x();
		timer3 = TIM3 ->CCR1;
		timer5 = TIM5 ->CCR1;
		timer10 = TIM10 -> CCR1;
		timer11 = TIM11 -> CCR1;
//		MPU6050_Read_Gyro  ();
//		delay();
	}
}


