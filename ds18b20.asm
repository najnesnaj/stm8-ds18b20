;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 11 2014) (Linux)
; This file was generated Wed Jul 19 15:02:31 2017
;--------------------------------------------------------
	.module ds18b20
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _segmentMap
	.globl _main
	.globl _DS18B20_ReadTemperature
	.globl _DS18B20_ReadByte
	.globl _DS18B20_WriteByte
	.globl _DS18B20_Init
	.globl __delay_ms
	.globl __delay_us
	.globl _tm1637DisplayDecimal
	.globl _tm1637Init
	.globl _InitializeUART
	.globl _InitializeI2C
	.globl _i2c_read_register
	.globl _print_byte_hex
	.globl _i2c_set_start_ack
	.globl _i2c_send_address
	.globl _UARTPrintF
	.globl _i2c_send_reg
	.globl _i2c_set_stop
	.globl _i2c_set_nak
	.globl _i2c_read
	.globl _delay
	.globl _InitializeSystemClock
	.globl _delayTenMicro
	.globl _tm1637SetBrightness
	.globl __tm1637Start
	.globl __tm1637Stop
	.globl __tm1637ReadResult
	.globl __tm1637WriteByte
	.globl __tm1637ClkHigh
	.globl __tm1637ClkLow
	.globl __tm1637DioHigh
	.globl __tm1637DioLow
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int 0x0000 ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	ds18b20.c: 33: void delayTenMicro (void) {
;	-----------------------------------------
;	 function delayTenMicro
;	-----------------------------------------
_delayTenMicro:
;	ds18b20.c: 35: for (a = 0; a < 50; ++a)
	ld	a, #0x32
00104$:
;	ds18b20.c: 36: __asm__("nop");
	nop
	dec	a
;	ds18b20.c: 35: for (a = 0; a < 50; ++a)
	tnz	a
	jrne	00104$
	ret
;	ds18b20.c: 39: void InitializeSystemClock() {
;	-----------------------------------------
;	 function InitializeSystemClock
;	-----------------------------------------
_InitializeSystemClock:
;	ds18b20.c: 40: CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
	ldw	x, #0x50c0
	clr	(x)
;	ds18b20.c: 41: CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
	ldw	x, #0x50c0
	ld	a, #0x01
	ld	(x), a
;	ds18b20.c: 42: CLK_ECKR = 0;                       //  Disable the external clock.
	ldw	x, #0x50c1
	clr	(x)
;	ds18b20.c: 43: while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
00101$:
	ldw	x, #0x50c0
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	ds18b20.c: 44: CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	ldw	x, #0x50c6
	clr	(x)
;	ds18b20.c: 45: CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
	ldw	x, #0x50c7
	ld	a, #0xff
	ld	(x), a
;	ds18b20.c: 46: CLK_PCKENR2 = 0xff;                 //  Ditto.
	ldw	x, #0x50ca
	ld	a, #0xff
	ld	(x), a
;	ds18b20.c: 47: CLK_CCOR = 0;                       //  Turn off CCO.
	ldw	x, #0x50c9
	clr	(x)
;	ds18b20.c: 48: CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	ldw	x, #0x50cc
	clr	(x)
;	ds18b20.c: 49: CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	ldw	x, #0x50cd
	clr	(x)
;	ds18b20.c: 50: CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
	ldw	x, #0x50c4
	ld	a, #0xe1
	ld	(x), a
;	ds18b20.c: 51: CLK_SWCR = 0;                       //  Reset the clock switch control register.
	ldw	x, #0x50c5
	clr	(x)
;	ds18b20.c: 52: CLK_SWCR = CLK_SWEN;                //  Enable switching.
	ldw	x, #0x50c5
	ld	a, #0x02
	ld	(x), a
;	ds18b20.c: 53: while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
00104$:
	ldw	x, #0x50c5
	ld	a, (x)
	srl	a
	jrc	00104$
	ret
;	ds18b20.c: 55: void delay (int time_ms) {
;	-----------------------------------------
;	 function delay
;	-----------------------------------------
_delay:
	sub	sp, #10
;	ds18b20.c: 57: for (x = 0; x < 1036*time_ms; ++x)
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x0c
	push	#0x04
	call	__mulint
	addw	sp, #4
	ldw	(0x09, sp), x
00103$:
	ldw	y, (0x09, sp)
	ldw	(0x07, sp), y
	ld	a, (0x07, sp)
	rlc	a
	clr	a
	sbc	a, #0x00
	ld	(0x06, sp), a
	ld	(0x05, sp), a
	ldw	x, (0x03, sp)
	cpw	x, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	ld	a, (0x01, sp)
	sbc	a, (0x05, sp)
	jrsge	00105$
;	ds18b20.c: 58: __asm__("nop");
	nop
;	ds18b20.c: 57: for (x = 0; x < 1036*time_ms; ++x)
	ldw	y, (0x03, sp)
	addw	y, #0x0001
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x03, sp), y
	ldw	(0x01, sp), x
	jra	00103$
00105$:
	addw	sp, #10
	ret
;	ds18b20.c: 60: void i2c_read (unsigned char *x) {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	ds18b20.c: 61: while ((I2C_SR1 & I2C_RXNE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	ds18b20.c: 62: *x = I2C_DR;
	ldw	y, (0x03, sp)
	ldw	x, #0x5216
	ld	a, (x)
	ld	(y), a
	ret
;	ds18b20.c: 64: void i2c_set_nak (void) {
;	-----------------------------------------
;	 function i2c_set_nak
;	-----------------------------------------
_i2c_set_nak:
;	ds18b20.c: 65: I2C_CR2 &= ~I2C_ACK;
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	ds18b20.c: 67: void i2c_set_stop (void) {
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	ds18b20.c: 68: I2C_CR2 |= I2C_STOP;
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	ds18b20.c: 70: void i2c_send_reg (UCHAR addr) {
;	-----------------------------------------
;	 function i2c_send_reg
;	-----------------------------------------
_i2c_send_reg:
	sub	sp, #2
;	ds18b20.c: 72: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	ds18b20.c: 73: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	ds18b20.c: 74: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x05, sp)
	ld	(x), a
;	ds18b20.c: 75: while ((I2C_SR1 & I2C_TXE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	sll	a
	jrnc	00101$
	addw	sp, #2
	ret
;	ds18b20.c: 79: void UARTPrintF (char *message) {
;	-----------------------------------------
;	 function UARTPrintF
;	-----------------------------------------
_UARTPrintF:
;	ds18b20.c: 80: char *ch = message;
	ldw	y, (0x03, sp)
;	ds18b20.c: 81: while (*ch) {
00104$:
	ld	a, (y)
	tnz	a
	jreq	00107$
;	ds18b20.c: 82: UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
	ldw	x, #0x5231
	ld	(x), a
;	ds18b20.c: 83: while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	sll	a
	jrnc	00101$
;	ds18b20.c: 84: ch++;                               //  Grab the next character.
	incw	y
	jra	00104$
00107$:
	ret
;	ds18b20.c: 90: void i2c_send_address (UCHAR addr, UCHAR mode) {
;	-----------------------------------------
;	 function i2c_send_address
;	-----------------------------------------
_i2c_send_address:
	sub	sp, #3
;	ds18b20.c: 92: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	ds18b20.c: 93: I2C_DR = (addr << 1) | mode;
	ld	a, (0x06, sp)
	sll	a
	or	a, (0x07, sp)
	ldw	x, #0x5216
	ld	(x), a
;	ds18b20.c: 94: if (mode == I2C_READ) {
	ld	a, (0x07, sp)
	cp	a, #0x01
	jrne	00127$
	ld	a, #0x01
	ld	(0x03, sp), a
	jra	00128$
00127$:
	clr	(0x03, sp)
00128$:
	tnz	(0x03, sp)
	jreq	00103$
;	ds18b20.c: 95: I2C_OARL = 0;
	ldw	x, #0x5213
	clr	(x)
;	ds18b20.c: 96: I2C_OARH = 0;
	ldw	x, #0x5214
	clr	(x)
;	ds18b20.c: 99: while ((I2C_SR1 & I2C_ADDR) == 0);
00103$:
;	ds18b20.c: 92: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	ds18b20.c: 99: while ((I2C_SR1 & I2C_ADDR) == 0);
	bcp	a, #0x02
	jreq	00103$
;	ds18b20.c: 100: if (mode == I2C_READ)
	tnz	(0x03, sp)
	jreq	00108$
;	ds18b20.c: 101: UNSET (I2C_SR1, I2C_ADDR);
	and	a, #0xfd
	ldw	x, #0x5217
	ld	(x), a
00108$:
	addw	sp, #3
	ret
;	ds18b20.c: 104: void i2c_set_start_ack (void) {
;	-----------------------------------------
;	 function i2c_set_start_ack
;	-----------------------------------------
_i2c_set_start_ack:
;	ds18b20.c: 105: I2C_CR2 = I2C_ACK | I2C_START;
	ldw	x, #0x5211
	ld	a, #0x05
	ld	(x), a
;	ds18b20.c: 106: while ((I2C_SR1 & I2C_SB) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	ds18b20.c: 113: void print_byte_hex (unsigned char buffer) {
;	-----------------------------------------
;	 function print_byte_hex
;	-----------------------------------------
_print_byte_hex:
	sub	sp, #12
;	ds18b20.c: 116: a = (buffer >> 4);
	ld	a, (0x0f, sp)
	swap	a
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	ds18b20.c: 117: if (a > 9)
	cpw	x, #0x0009
	jrsle	00102$
;	ds18b20.c: 118: a = a + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x03, sp), x
	jra	00103$
00102$:
;	ds18b20.c: 120: a += '0'; 
	addw	x, #0x0030
	ldw	(0x03, sp), x
00103$:
;	ds18b20.c: 121: b = buffer & 0x0f;
	ld	a, (0x0f, sp)
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	ds18b20.c: 122: if (b > 9)
	cpw	x, #0x0009
	jrsle	00105$
;	ds18b20.c: 123: b = b + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x01, sp), x
	jra	00106$
00105$:
;	ds18b20.c: 125: b += '0'; 
	addw	x, #0x0030
	ldw	(0x01, sp), x
00106$:
;	ds18b20.c: 126: message[0] = a;
	ldw	y, sp
	addw	y, #5
	ld	a, (0x04, sp)
	ld	(y), a
;	ds18b20.c: 127: message[1] = b;
	ldw	x, y
	incw	x
	ld	a, (0x02, sp)
	ld	(x), a
;	ds18b20.c: 128: message[2] = 0;
	ldw	x, y
	incw	x
	incw	x
	clr	(x)
;	ds18b20.c: 129: UARTPrintF (message);
	pushw	y
	call	_UARTPrintF
	addw	sp, #2
	addw	sp, #12
	ret
;	ds18b20.c: 133: unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
;	-----------------------------------------
;	 function i2c_read_register
;	-----------------------------------------
_i2c_read_register:
	sub	sp, #2
;	ds18b20.c: 136: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	ds18b20.c: 137: i2c_send_address (addr, I2C_WRITE);
	push	#0x00
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	ds18b20.c: 138: i2c_send_reg (rg);
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_reg
	pop	a
;	ds18b20.c: 139: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	ds18b20.c: 140: i2c_send_address (addr, I2C_READ);
	push	#0x01
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	ds18b20.c: 141: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	(0x01, sp), a
;	ds18b20.c: 142: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	(0x01, sp), a
;	ds18b20.c: 143: i2c_set_nak ();
	call	_i2c_set_nak
;	ds18b20.c: 144: i2c_set_stop ();
	call	_i2c_set_stop
;	ds18b20.c: 145: i2c_read (&x);
	ldw	x, sp
	incw	x
	incw	x
	pushw	x
	call	_i2c_read
	addw	sp, #2
;	ds18b20.c: 146: return (x);
	ld	a, (0x02, sp)
	addw	sp, #2
	ret
;	ds18b20.c: 149: void InitializeI2C (void) {
;	-----------------------------------------
;	 function InitializeI2C
;	-----------------------------------------
_InitializeI2C:
;	ds18b20.c: 150: I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
	ldw	x, #0x5210
	clr	(x)
;	ds18b20.c: 154: I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
	ldw	x, #0x5212
	ld	a, #0x10
	ld	(x), a
;	ds18b20.c: 155: UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
	bres	0x521c, #7
;	ds18b20.c: 157: I2C_CCRL = 0xa0;                    //  SCL clock speed is 50 kHz.
	ldw	x, #0x521b
	ld	a, #0xa0
	ld	(x), a
;	ds18b20.c: 159: I2C_CCRH &= 0x00;	// Clears lower 4 bits "CCR"
	ldw	x, #0x521c
	clr	(x)
;	ds18b20.c: 163: UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
	bres	0x5214, #7
;	ds18b20.c: 164: SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
	ldw	x, #0x5214
	ld	a, (x)
	or	a, #0x40
	ld	(x), a
;	ds18b20.c: 168: I2C_TRISER = 17;
	ldw	x, #0x521d
	ld	a, #0x11
	ld	(x), a
;	ds18b20.c: 176: I2C_CR1 = I2C_PE;	// Enables port
	ldw	x, #0x5210
	ld	a, #0x01
	ld	(x), a
	ret
;	ds18b20.c: 182: void InitializeUART() {
;	-----------------------------------------
;	 function InitializeUART
;	-----------------------------------------
_InitializeUART:
;	ds18b20.c: 192: UART1_CR1 = 0;
	ldw	x, #0x5234
	clr	(x)
;	ds18b20.c: 193: UART1_CR2 = 0;
	ldw	x, #0x5235
	clr	(x)
;	ds18b20.c: 194: UART1_CR4 = 0;
	ldw	x, #0x5237
	clr	(x)
;	ds18b20.c: 195: UART1_CR3 = 0;
	ldw	x, #0x5236
	clr	(x)
;	ds18b20.c: 196: UART1_CR5 = 0;
	ldw	x, #0x5238
	clr	(x)
;	ds18b20.c: 197: UART1_GTR = 0;
	ldw	x, #0x5239
	clr	(x)
;	ds18b20.c: 198: UART1_PSCR = 0;
	ldw	x, #0x523a
	clr	(x)
;	ds18b20.c: 202: UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	ds18b20.c: 203: UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	ds18b20.c: 204: UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	ds18b20.c: 205: UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	ds18b20.c: 206: UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
	ldw	x, #0x5233
	ld	a, #0x0a
	ld	(x), a
;	ds18b20.c: 207: UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
	ldw	x, #0x5232
	ld	a, #0x08
	ld	(x), a
;	ds18b20.c: 211: UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 212: UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	ds18b20.c: 216: SET (UART1_CR3, CR3_CPOL);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	ds18b20.c: 217: SET (UART1_CR3, CR3_CPHA);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	ds18b20.c: 218: SET (UART1_CR3, CR3_LBCL);
	bset	0x5236, #0
;	ds18b20.c: 222: SET (UART1_CR2, CR2_TEN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 223: SET (UART1_CR2, CR2_REN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	ds18b20.c: 224: UART1_CR3 = CR3_CLKEN;
	ldw	x, #0x5236
	ld	a, #0x08
	ld	(x), a
	ret
;	ds18b20.c: 252: void tm1637Init(void)
;	-----------------------------------------
;	 function tm1637Init
;	-----------------------------------------
_tm1637Init:
;	ds18b20.c: 254: tm1637SetBrightness(8);
	push	#0x08
	call	_tm1637SetBrightness
	pop	a
	ret
;	ds18b20.c: 259: void tm1637DisplayDecimal(long TT,unsigned int displaySeparator)
;	-----------------------------------------
;	 function tm1637DisplayDecimal
;	-----------------------------------------
_tm1637DisplayDecimal:
	sub	sp, #19
;	ds18b20.c: 261: unsigned int v = TT & 0x0000FFFF;
	ld	a, (0x19, sp)
	ld	xl, a
	ld	a, (0x18, sp)
	ld	xh, a
	clr	(0x11, sp)
	clr	a
	ldw	(0x05, sp), x
;	ds18b20.c: 267: for (ii = 0; ii < 4; ++ii) {
	ldw	x, sp
	incw	x
	ldw	(0x0a, sp), x
	ldw	x, #_segmentMap+0
	ldw	(0x0e, sp), x
	clrw	y
00106$:
;	ds18b20.c: 268: digitArr[ii] = segmentMap[v % 10];
	ldw	x, y
	addw	x, (0x0a, sp)
	ldw	(0x0c, sp), x
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	x, y
	popw	y
	addw	x, (0x0e, sp)
	ld	a, (x)
	ldw	x, (0x0c, sp)
	ld	(x), a
;	ds18b20.c: 269: if (ii == 2 && displaySeparator) {
	cpw	y, #0x0002
	jrne	00102$
	ldw	x, (0x1a, sp)
	jreq	00102$
;	ds18b20.c: 270: digitArr[ii] |= 1 << 7;
	ldw	x, (0x0c, sp)
	ld	a, (x)
	or	a, #0x80
	ldw	x, (0x0c, sp)
	ld	(x), a
00102$:
;	ds18b20.c: 272: v /= 10;
	pushw	y
	ldw	x, (0x07, sp)
	ldw	y, #0x000a
	divw	x, y
	popw	y
	ldw	(0x05, sp), x
;	ds18b20.c: 267: for (ii = 0; ii < 4; ++ii) {
	incw	y
	cpw	y, #0x0004
	jrc	00106$
;	ds18b20.c: 275: _tm1637Start();
	call	__tm1637Start
;	ds18b20.c: 276: _tm1637WriteByte(0x40);
	push	#0x40
	call	__tm1637WriteByte
	pop	a
;	ds18b20.c: 277: _tm1637ReadResult();
	call	__tm1637ReadResult
;	ds18b20.c: 278: _tm1637Stop();
	call	__tm1637Stop
;	ds18b20.c: 280: _tm1637Start();
	call	__tm1637Start
;	ds18b20.c: 281: _tm1637WriteByte(0xc0);
	push	#0xc0
	call	__tm1637WriteByte
	pop	a
;	ds18b20.c: 282: _tm1637ReadResult();
	call	__tm1637ReadResult
;	ds18b20.c: 284: for (ii = 0; ii < 4; ++ii) {
	clrw	x
	ldw	(0x07, sp), x
00108$:
;	ds18b20.c: 285: _tm1637WriteByte(digitArr[3 - ii]);
	ld	a, (0x08, sp)
	ld	(0x09, sp), a
	ld	a, #0x03
	sub	a, (0x09, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x0a, sp)
	ld	a, (x)
	push	a
	call	__tm1637WriteByte
	pop	a
;	ds18b20.c: 286: _tm1637ReadResult();
	call	__tm1637ReadResult
;	ds18b20.c: 284: for (ii = 0; ii < 4; ++ii) {
	ldw	x, (0x07, sp)
	incw	x
	ldw	(0x07, sp), x
	ldw	x, (0x07, sp)
	cpw	x, #0x0004
	jrc	00108$
;	ds18b20.c: 289: _tm1637Stop();
	call	__tm1637Stop
	addw	sp, #19
	ret
;	ds18b20.c: 294: void tm1637SetBrightness(char brightness)
;	-----------------------------------------
;	 function tm1637SetBrightness
;	-----------------------------------------
_tm1637SetBrightness:
;	ds18b20.c: 301: _tm1637Start();
	call	__tm1637Start
;	ds18b20.c: 302: _tm1637WriteByte(0x87 + brightness);
	ld	a, (0x03, sp)
	add	a, #0x87
	push	a
	call	__tm1637WriteByte
	pop	a
;	ds18b20.c: 303: _tm1637ReadResult();
	call	__tm1637ReadResult
;	ds18b20.c: 304: _tm1637Stop();
	jp	__tm1637Stop
;	ds18b20.c: 307: void _tm1637Start(void)
;	-----------------------------------------
;	 function _tm1637Start
;	-----------------------------------------
__tm1637Start:
;	ds18b20.c: 309: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	ds18b20.c: 310: _tm1637DioHigh();
	call	__tm1637DioHigh
;	ds18b20.c: 311: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 312: _tm1637DioLow();
	jp	__tm1637DioLow
;	ds18b20.c: 315: void _tm1637Stop(void)
;	-----------------------------------------
;	 function _tm1637Stop
;	-----------------------------------------
__tm1637Stop:
;	ds18b20.c: 317: _tm1637ClkLow();
	call	__tm1637ClkLow
;	ds18b20.c: 318: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 319: _tm1637DioLow();
	call	__tm1637DioLow
;	ds18b20.c: 320: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 321: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	ds18b20.c: 322: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 323: _tm1637DioHigh();
	jp	__tm1637DioHigh
;	ds18b20.c: 326: void _tm1637ReadResult(void)
;	-----------------------------------------
;	 function _tm1637ReadResult
;	-----------------------------------------
__tm1637ReadResult:
;	ds18b20.c: 328: _tm1637ClkLow();
	call	__tm1637ClkLow
;	ds18b20.c: 329: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 331: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	ds18b20.c: 332: delay(5);
	push	#0x05
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 333: _tm1637ClkLow();
	jp	__tm1637ClkLow
;	ds18b20.c: 336: void _tm1637WriteByte(unsigned char b)
;	-----------------------------------------
;	 function _tm1637WriteByte
;	-----------------------------------------
__tm1637WriteByte:
	sub	sp, #2
;	ds18b20.c: 338: for (ii = 0; ii < 8; ++ii) {
	clrw	x
	ldw	(0x01, sp), x
00105$:
;	ds18b20.c: 339: _tm1637ClkLow();
	call	__tm1637ClkLow
;	ds18b20.c: 340: if (b & 0x01) {
	ld	a, (0x05, sp)
	srl	a
	jrnc	00102$
;	ds18b20.c: 341: _tm1637DioHigh();
	call	__tm1637DioHigh
	jra	00103$
00102$:
;	ds18b20.c: 344: _tm1637DioLow();
	call	__tm1637DioLow
00103$:
;	ds18b20.c: 346: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 347: b >>= 1;
	ld	a, (0x05, sp)
	srl	a
	ld	(0x05, sp), a
;	ds18b20.c: 348: _tm1637ClkHigh();
	call	__tm1637ClkHigh
;	ds18b20.c: 349: delay(15);
	push	#0x0f
	push	#0x00
	call	_delay
	addw	sp, #2
;	ds18b20.c: 338: for (ii = 0; ii < 8; ++ii) {
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	ldw	x, (0x01, sp)
	cpw	x, #0x0008
	jrslt	00105$
	addw	sp, #2
	ret
;	ds18b20.c: 355: void _tm1637ClkHigh(void)
;	-----------------------------------------
;	 function _tm1637ClkHigh
;	-----------------------------------------
__tm1637ClkHigh:
;	ds18b20.c: 360: PD_ODR |= 1 << 2;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	ds18b20.c: 363: void _tm1637ClkLow(void)
;	-----------------------------------------
;	 function _tm1637ClkLow
;	-----------------------------------------
__tm1637ClkLow:
;	ds18b20.c: 367: PD_ODR &= ~(1 << 2);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	ds18b20.c: 373: void _tm1637DioHigh(void)
;	-----------------------------------------
;	 function _tm1637DioHigh
;	-----------------------------------------
__tm1637DioHigh:
;	ds18b20.c: 377: PD_ODR |= 1 << 3;
	ldw	x, #0x500f
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	ds18b20.c: 381: void _tm1637DioLow(void)
;	-----------------------------------------
;	 function _tm1637DioLow
;	-----------------------------------------
__tm1637DioLow:
;	ds18b20.c: 383: PD_ODR &= ~(1 << 3);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	ds18b20.c: 391: void _delay_us(unsigned int i)
;	-----------------------------------------
;	 function _delay_us
;	-----------------------------------------
__delay_us:
;	ds18b20.c: 393: i *= 3; 
	ldw	x, (0x03, sp)
	pushw	x
	push	#0x03
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	(0x03, sp), x
;	ds18b20.c: 394: while(--i);
	ldw	x, (0x03, sp)
00101$:
	decw	x
	tnzw	x
	jrne	00101$
	ret
;	ds18b20.c: 397: void _delay_ms(unsigned int i)
;	-----------------------------------------
;	 function _delay_ms
;	-----------------------------------------
__delay_ms:
;	ds18b20.c: 399: while(i--)
	ldw	x, (0x03, sp)
00101$:
	ldw	y, x
	decw	x
	tnzw	y
	jreq	00104$
;	ds18b20.c: 401: _delay_us(1000);
	pushw	x
	push	#0xe8
	push	#0x03
	call	__delay_us
	addw	sp, #2
	popw	x
	jra	00101$
00104$:
	ret
;	ds18b20.c: 407: void DS18B20_Init(void)
;	-----------------------------------------
;	 function DS18B20_Init
;	-----------------------------------------
_DS18B20_Init:
;	ds18b20.c: 409: DS18B20_DQ_OUT;   
	ldw	x, #0x5002
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 410: DS18B20_DQ_PUSH_PULL;    
	ldw	x, #0x5003
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 411: DS18B20_DQ_HIGH;   
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 412: _delay_us(10);
	push	#0x0a
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 413: DS18B20_DQ_LOW;   
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 414: _delay_us(600);     //????
	push	#0x58
	push	#0x02
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 416: DS18B20_DQ_IN;   
	ldw	x, #0x5002
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 417: DS18B20_DQ_PULL_UP;    
	ldw	x, #0x5003
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 418: _delay_us(100);     
	push	#0x64
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 419: while(DS18B20_DQ_VALUE == 1);
00101$:
	ldw	x, #0x5001
	ld	a, (x)
	cp	a, #0x01
	jreq	00101$
;	ds18b20.c: 420: _delay_us(400);
	push	#0x90
	push	#0x01
	call	__delay_us
	addw	sp, #2
	ret
;	ds18b20.c: 424: void DS18B20_WriteByte(unsigned char _data)
;	-----------------------------------------
;	 function DS18B20_WriteByte
;	-----------------------------------------
_DS18B20_WriteByte:
	push	a
;	ds18b20.c: 428: DS18B20_DQ_OUT;
	ldw	x, #0x5002
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 429: for (i = 0; i < 8; i++)
	clr	(0x01, sp)
00104$:
;	ds18b20.c: 431: DS18B20_DQ_LOW;
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 432: _delay_us(2);
	push	#0x02
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 433: if (_data & 0x01)
	ld	a, (0x04, sp)
	srl	a
	jrnc	00102$
;	ds18b20.c: 435: DS18B20_DQ_HIGH;
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
00102$:
;	ds18b20.c: 437: _data >>= 1;
	ld	a, (0x04, sp)
	srl	a
	ld	(0x04, sp), a
;	ds18b20.c: 438: _delay_us(60);
	push	#0x3c
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 439: DS18B20_DQ_HIGH;
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 429: for (i = 0; i < 8; i++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x08
	jrc	00104$
	pop	a
	ret
;	ds18b20.c: 443: unsigned char DS18B20_ReadByte(void)
;	-----------------------------------------
;	 function DS18B20_ReadByte
;	-----------------------------------------
_DS18B20_ReadByte:
	sub	sp, #2
;	ds18b20.c: 445: unsigned char i = 0, _data = 0;
	clr	(0x01, sp)
;	ds18b20.c: 447: for (i = 0; i < 8; i++)
	clr	(0x02, sp)
00104$:
;	ds18b20.c: 449: DS18B20_DQ_OUT;
	ldw	x, #0x5002
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 450: DS18B20_DQ_LOW;
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 451: _delay_us(5);
	push	#0x05
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 452: _data >>= 1;
	srl	(0x01, sp)
;	ds18b20.c: 453: DS18B20_DQ_HIGH;
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 454: DS18B20_DQ_IN;
	ldw	x, #0x5002
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	ds18b20.c: 455: if (DS18B20_DQ_VALUE)
	ldw	x, #0x5001
	ld	a, (x)
	tnz	a
	jreq	00102$
;	ds18b20.c: 457: _data |= 0x80;
	ld	a, (0x01, sp)
	or	a, #0x80
	ld	(0x01, sp), a
00102$:
;	ds18b20.c: 459: DS18B20_DQ_OUT; 
	ldw	x, #0x5002
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 460: DS18B20_DQ_HIGH;
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	ds18b20.c: 461: _delay_us(60);
	push	#0x3c
	push	#0x00
	call	__delay_us
	addw	sp, #2
;	ds18b20.c: 447: for (i = 0; i < 8; i++)
	inc	(0x02, sp)
	ld	a, (0x02, sp)
	cp	a, #0x08
	jrc	00104$
;	ds18b20.c: 464: return _data;
	ld	a, (0x01, sp)
	addw	sp, #2
	ret
;	ds18b20.c: 467: float DS18B20_ReadTemperature(void)
;	-----------------------------------------
;	 function DS18B20_ReadTemperature
;	-----------------------------------------
_DS18B20_ReadTemperature:
	sub	sp, #14
;	ds18b20.c: 472: DS18B20_Init();
	call	_DS18B20_Init
;	ds18b20.c: 473: DS18B20_WriteByte(0xcc);
	push	#0xcc
	call	_DS18B20_WriteByte
	pop	a
;	ds18b20.c: 474: DS18B20_WriteByte(0x44);
	push	#0x44
	call	_DS18B20_WriteByte
	pop	a
;	ds18b20.c: 476: DS18B20_Init();
	call	_DS18B20_Init
;	ds18b20.c: 477: DS18B20_WriteByte(0xcc);
	push	#0xcc
	call	_DS18B20_WriteByte
	pop	a
;	ds18b20.c: 478: DS18B20_WriteByte(0xbe);
	push	#0xbe
	call	_DS18B20_WriteByte
	pop	a
;	ds18b20.c: 480: temp = DS18B20_ReadByte();
	call	_DS18B20_ReadByte
;	ds18b20.c: 481: t = (((temp & 0xf0) >> 4) + (temp & 0x07) * 0.125); 
	ld	xh, a
	and	a, #0xf0
	swap	a
	and	a, #0x0f
	ld	(0x0e, sp), a
	clr	(0x0d, sp)
	ld	a, xh
	and	a, #0x07
	push	a
	call	___uchar2fs
	pop	a
	pushw	x
	pushw	y
	clrw	x
	pushw	x
	push	#0x00
	push	#0x3e
	call	___fsmul
	addw	sp, #8
	ldw	(0x05, sp), y
	pushw	x
	ldw	y, (0x0f, sp)
	pushw	y
	call	___sint2fs
	addw	sp, #2
	ldw	(0x05, sp), x
	ldw	x, (0x07, sp)
	pushw	x
	ldw	x, (0x07, sp)
	pushw	x
	pushw	y
	call	___fsadd
	addw	sp, #8
	ldw	(0x0b, sp), x
	ldw	(0x09, sp), y
;	ds18b20.c: 482: temp = DS18B20_ReadByte();
	call	_DS18B20_ReadByte
;	ds18b20.c: 483: t += ((temp & 0x0f) << 4);
	and	a, #0x0f
	clrw	x
	ld	xl, a
	sllw	x
	sllw	x
	sllw	x
	sllw	x
	pushw	x
	call	___sint2fs
	addw	sp, #2
	pushw	x
	pushw	y
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	___fsadd
	addw	sp, #8
;	ds18b20.c: 485: return t;
	addw	sp, #14
	ret
;	ds18b20.c: 491: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #22
;	ds18b20.c: 495: InitializeSystemClock();
	call	_InitializeSystemClock
;	ds18b20.c: 498: PD_DDR = (1 << 3) | (1 << 2); // output mode
	ldw	x, #0x5011
	ld	a, #0x0c
	ld	(x), a
;	ds18b20.c: 499: PD_CR1 = (1 << 3) | (1 << 2); // push-pull
	ldw	x, #0x5012
	ld	a, #0x0c
	ld	(x), a
;	ds18b20.c: 500: PD_CR2 = (1 << 3) | (1 << 2); // up to 10MHz speed
	ldw	x, #0x5013
	ld	a, #0x0c
	ld	(x), a
;	ds18b20.c: 501: tm1637Init();
	call	_tm1637Init
;	ds18b20.c: 503: InitializeUART();
	call	_InitializeUART
;	ds18b20.c: 507: while (1) {
00114$:
;	ds18b20.c: 510: objTemp = DS18B20_ReadTemperature(); 
	call	_DS18B20_ReadTemperature
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
;	ds18b20.c: 513: while (objTemp > 1000) {
	clrw	x
	ldw	(0x01, sp), x
00101$:
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00127$
;	ds18b20.c: 514: vierde+=1;
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
;	ds18b20.c: 515: objTemp-=1000;
	clrw	x
	pushw	x
	push	#0x7a
	push	#0x44
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	jra	00101$
;	ds18b20.c: 517: while (objTemp > 100) {
00127$:
	ldw	y, (0x01, sp)
	ldw	(0x15, sp), y
	clrw	x
	ldw	(0x03, sp), x
00104$:
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00128$
;	ds18b20.c: 518: derde+=1;
	ldw	x, (0x03, sp)
	incw	x
	ldw	(0x03, sp), x
;	ds18b20.c: 519: objTemp-=100;
	clrw	x
	pushw	x
	push	#0xc8
	push	#0x42
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	jra	00104$
;	ds18b20.c: 521: while (objTemp > 10) {
00128$:
	ldw	y, (0x03, sp)
	ldw	(0x13, sp), y
	clrw	x
	ldw	(0x0b, sp), x
00107$:
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00129$
;	ds18b20.c: 522: tweede+=1;
	ldw	x, (0x0b, sp)
	incw	x
	ldw	(0x0b, sp), x
;	ds18b20.c: 523: objTemp-=10;
	clrw	x
	pushw	x
	push	#0x20
	push	#0x41
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	jra	00107$
;	ds18b20.c: 525: while (objTemp > 0)
00129$:
	ldw	y, (0x0b, sp)
	ldw	(0x11, sp), y
	clrw	x
	ldw	(0x09, sp), x
00110$:
	clrw	x
	pushw	x
	clrw	x
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fsgt
	addw	sp, #8
	tnz	a
	jreq	00112$
;	ds18b20.c: 527: eerste+=1;
	ldw	x, (0x09, sp)
	incw	x
	ldw	(0x09, sp), x
;	ds18b20.c: 528: objTemp-=1;
	clrw	x
	pushw	x
	push	#0x80
	push	#0x3f
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	___fssub
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	jra	00110$
00112$:
;	ds18b20.c: 531: utemp=vierde*1000+derde*100+tweede*10+eerste;
	ldw	x, (0x15, sp)
	pushw	x
	push	#0xe8
	push	#0x03
	call	__mulint
	addw	sp, #4
	ldw	(0x0f, sp), x
	ldw	x, (0x13, sp)
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x0f, sp)
	ldw	(0x0d, sp), x
	ldw	x, (0x11, sp)
	pushw	x
	push	#0x0a
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x0d, sp)
	addw	x, (0x09, sp)
	clrw	y
	tnzw	x
	jrpl	00162$
	decw	y
00162$:
;	ds18b20.c: 534: tm1637DisplayDecimal(utemp, 1); // eg 37:12
	push	#0x01
	push	#0x00
	pushw	x
	pushw	y
	call	_tm1637DisplayDecimal
	addw	sp, #6
;	ds18b20.c: 537: delayTenMicro();
	call	_delayTenMicro
	jp	00114$
	addw	sp, #22
	ret
	.area CODE
_segmentMap:
	.db #0x3F	;  63
	.db #0x06	;  6
	.db #0x5B	;  91
	.db #0x4F	;  79	'O'
	.db #0x66	;  102	'f'
	.db #0x6D	;  109	'm'
	.db #0x7D	;  125
	.db #0x07	;  7
	.db #0x7F	;  127
	.db #0x6F	;  111	'o'
	.db #0x77	;  119	'w'
	.db #0x7C	;  124
	.db #0x39	;  57	'9'
	.db #0x5E	;  94
	.db #0x79	;  121	'y'
	.db #0x71	;  113	'q'
	.db #0x00	;  0
	.area INITIALIZER
	.area CABS (ABS)
