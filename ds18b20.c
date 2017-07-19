#include "stm8.h"
#include <string.h>
#define SET(x, y)   (x) |= (y)
#define UNSET(x, y) (x) &= ~(y)
#define READ(x, y)  ((x) & (y))
#define MLX90614_ADDR	0x5a
#define I2C_READ        1
#define I2C_WRITE       0





typedef unsigned char UCHAR;
void delayTenMicro (void) {
	char a;
	for (a = 0; a < 50; ++a)
		__asm__("nop");
}

void InitializeSystemClock() {
	CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
	CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
	CLK_ECKR = 0;                       //  Disable the external clock.
	while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
	CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
	CLK_PCKENR2 = 0xff;                 //  Ditto.
	CLK_CCOR = 0;                       //  Turn off CCO.
	CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
	CLK_SWCR = 0;                       //  Reset the clock switch control register.
	CLK_SWCR = CLK_SWEN;                //  Enable switching.
	while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
}
void delay (int time_ms) {
	volatile long int x;
	for (x = 0; x < 1036*time_ms; ++x)
		__asm__("nop");
}
void i2c_read (unsigned char *x) {
	while ((I2C_SR1 & I2C_RXNE) == 0);
	*x = I2C_DR;
}
void i2c_set_nak (void) {
	I2C_CR2 &= ~I2C_ACK;
}
void i2c_set_stop (void) {
	I2C_CR2 |= I2C_STOP;
}
void i2c_send_reg (UCHAR addr) {
	volatile int reg;
	reg = I2C_SR1;
	reg = I2C_SR3;
	I2C_DR = addr;
	while ((I2C_SR1 & I2C_TXE) == 0);
}


void UARTPrintF (char *message) {
	char *ch = message;
	while (*ch) {
		UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
		while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
		ch++;                               //  Grab the next character.
	}
}



void i2c_send_address (UCHAR addr, UCHAR mode) {
	volatile int reg;
	reg = I2C_SR1;
	I2C_DR = (addr << 1) | mode;
	if (mode == I2C_READ) {
		I2C_OARL = 0;
		I2C_OARH = 0;
	}

	while ((I2C_SR1 & I2C_ADDR) == 0);
	if (mode == I2C_READ)
		UNSET (I2C_SR1, I2C_ADDR);
}

void i2c_set_start_ack (void) {
	I2C_CR2 = I2C_ACK | I2C_START;
	while ((I2C_SR1 & I2C_SB) == 0);
}

//
//  Send a message to the debug port (UART1).
//

void print_byte_hex (unsigned char buffer) {
	unsigned char message[8];
	int a, b;
	a = (buffer >> 4);
	if (a > 9)
		a = a + 'a' - 10;
	else
		a += '0'; 
	b = buffer & 0x0f;
	if (b > 9)
		b = b + 'a' - 10;
	else
		b += '0'; 
	message[0] = a;
	message[1] = b;
	message[2] = 0;
	UARTPrintF (message);
}


unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
	volatile UCHAR reg;
	UCHAR x;
	i2c_set_start_ack ();
	i2c_send_address (addr, I2C_WRITE);
	i2c_send_reg (rg);
	i2c_set_start_ack ();
	i2c_send_address (addr, I2C_READ);
	reg = I2C_SR1;
	reg = I2C_SR3;
	i2c_set_nak ();
	i2c_set_stop ();
	i2c_read (&x);
	return (x);
}

void InitializeI2C (void) {
	I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
	//
	//  Setup the clock information.
	//
	I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
	UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
	//	I2C_CCRL = 0x10;                    //  SCL clock speed is 500 kHz.
	I2C_CCRL = 0xa0;                    //  SCL clock speed is 50 kHz.
	//		I2C_CCRH &= 0xf0;	// Clears lower 4 bits "CCR"
	I2C_CCRH &= 0x00;	// Clears lower 4 bits "CCR"
	//
	//  Set the address of this device.
	//
	UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
	SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
	//
	//  Setup the bus characteristics.
	//
	I2C_TRISER = 17;
	//
	//  Turn on the interrupts.
	//
	//I2C_ITR = I2C_ITBUFEN | I2C_ITEVTEN | I2C_ITERREN; //  Buffer, event and error interrupts enabled
	//
	//  Configuration complete so turn the peripheral on.
	//
	I2C_CR1 = I2C_PE;	// Enables port
	//
	//  Enter master mode.
	//
}

void InitializeUART() {
	//
	//  Clear the Idle Line Detected bit in the status register by a read
	//  to the UART1_SR register followed by a Read to the UART1_DR register.
	//
	unsigned char tmp = UART1_SR;
	tmp = UART1_DR;
	//
	//  Reset the UART registers to the reset values.
	//
	UART1_CR1 = 0;
	UART1_CR2 = 0;
	UART1_CR4 = 0;
	UART1_CR3 = 0;
	UART1_CR5 = 0;
	UART1_GTR = 0;
	UART1_PSCR = 0;
	//
	//  Now setup the port to 115200,n,8,1.
	//
	UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
	UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
	UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
	UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
	UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
	UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
	//
	//  Disable the transmitter and receiver.
	//
	UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
	UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
	//
	//  Set the clock polarity, lock phase and last bit clock pulse.
	//
	SET (UART1_CR3, CR3_CPOL);
	SET (UART1_CR3, CR3_CPHA);
	SET (UART1_CR3, CR3_LBCL);
	//
	//  Turn on the UART transmit, receive and the UART clock.
	//
	SET (UART1_CR2, CR2_TEN);
	SET (UART1_CR2, CR2_REN);
	UART1_CR3 = CR3_CLKEN;
}


// DISPLAY




void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);

void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);
void tm1637SetBrightness(char brightness);

const char segmentMap[] = {
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
	0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
	0x00
};



void tm1637Init(void)
{
	tm1637SetBrightness(8);
}



void tm1637DisplayDecimal(long TT,unsigned int displaySeparator)
{ unsigned int ii;
	unsigned int v = TT & 0x0000FFFF;
	unsigned char digitArr[4];



	//  unsigned char digitArr[4];
	for (ii = 0; ii < 4; ++ii) {
		digitArr[ii] = segmentMap[v % 10];
		if (ii == 2 && displaySeparator) {
			digitArr[ii] |= 1 << 7;
		}
		v /= 10;
	}

	_tm1637Start();
	_tm1637WriteByte(0x40);
	_tm1637ReadResult();
	_tm1637Stop();

	_tm1637Start();
	_tm1637WriteByte(0xc0);
	_tm1637ReadResult();

	for (ii = 0; ii < 4; ++ii) {
		_tm1637WriteByte(digitArr[3 - ii]);
		_tm1637ReadResult();
	}

	_tm1637Stop();
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(char brightness)
{
	// Brightness command:
	// 1000 0XXX = display off
	// 1000 1BBB = display on, brightness 0-7
	// X = don't care
	// B = brightness
	_tm1637Start();
	_tm1637WriteByte(0x87 + brightness);
	_tm1637ReadResult();
	_tm1637Stop();
}

void _tm1637Start(void)
{
	_tm1637ClkHigh();
	_tm1637DioHigh();
	delay(5);
	_tm1637DioLow();
}

void _tm1637Stop(void)
{
	_tm1637ClkLow();
	delay(5);
	_tm1637DioLow();
	delay(5);
	_tm1637ClkHigh();
	delay(5);
	_tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
	_tm1637ClkLow();
	delay(5);
	// while (dio); // We're cheating here and not actually reading back the response.
	_tm1637ClkHigh();
	delay(5);
	_tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{int ii;
	for (ii = 0; ii < 8; ++ii) {
		_tm1637ClkLow();
		if (b & 0x01) {
			_tm1637DioHigh();
		}
		else {
			_tm1637DioLow();
		}
		delay(15);
		b >>= 1;
		_tm1637ClkHigh();
		delay(15);
	}
}



void _tm1637ClkHigh(void)
{ 
	//PB_ODR_bit.ODR5 = 1; //      _tm1637ClkHigh(); 

	//  GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
	PD_ODR |= 1 << 2;
}

void _tm1637ClkLow(void)
{ 
	// GPIO_WriteLow(GPIOD,GPIO_PIN_2);

	PD_ODR &= ~(1 << 2);

	//    PB_ODR_bit.ODR5 = 0; //      _tm1637ClkHigh(); 

}

void _tm1637DioHigh(void)
{
	//PB_ODR_bit.ODR4 = 1; //  _tm1637DioHigh(); 
	// GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
	PD_ODR |= 1 << 3;

}

void _tm1637DioLow(void)
{
	PD_ODR &= ~(1 << 3);

	//GPIO_WriteLow(GPIOD,GPIO_PIN_3);
	//PB_ODR_bit.ODR4 = 0; //  _tm1637DioHigh(); 

}








int main () {
	char p1,p2;
	int eerste, tweede, derde, vierde;
	long utemp;
	float objTemp;
	UCHAR  x;
	volatile int reg;
	InitializeSystemClock();

	//display on PD2 PD3
	PD_DDR = (1 << 3) | (1 << 2); // output mode
	PD_CR1 = (1 << 3) | (1 << 2); // push-pull
	PD_CR2 = (1 << 3) | (1 << 2); // up to 10MHz speed
	tm1637Init();

	InitializeUART();
	InitializeI2C();


	while (1) {

		i2c_set_start_ack ();
		i2c_send_address (MLX90614_ADDR, I2C_WRITE);
		i2c_send_reg(0x07); // object temperature		
		//register for ambiant temperature	i2c_send_reg(0x06);		
		i2c_set_start_ack ();
		i2c_send_address (MLX90614_ADDR, I2C_READ);
		reg = I2C_SR1;
		reg = I2C_SR3;

		i2c_set_nak();

		i2c_read (&x);
		p1=x;
		i2c_read (&x);
		p2=x;
		i2c_set_stop ();

		objTemp = ((((p2&0x007f)<<8)+p1)*2)-27315; //subtract kelvin for celcius
		eerste=0;tweede=0;derde=0;vierde=0;
		//make measurement suitable for display
		while (objTemp > 1000) {
			vierde+=1;
			objTemp-=1000;
		} 
		while (objTemp > 100) {
			derde+=1;
			objTemp-=100;
		} 
		while (objTemp > 10) {
			tweede+=1;
			objTemp-=10;
		}
		while (objTemp > 0)
		{
			eerste+=1;
			objTemp-=1;
		}

		utemp=vierde*1000+derde*100+tweede*10+eerste;


		tm1637DisplayDecimal(utemp, 1); // eg 37:12


		delayTenMicro();
	}
}
