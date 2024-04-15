// #include <LiquidCrystal.h>


#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;



void setup() {
  // put your setup code here, to run once:
  // initialize the serial port on USART0:
 U0init(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned char cs1;
  while (U0kbhit()==0){}; // wait for RDA = true
  cs1 = U0getchar();    // read character

  readAndPrint(cs1);
}

void readAndPrint(unsigned char input) {
  // unsigned char cs1;
  // while (U0kbhit()==0){}; // wait for RDA = true
  // cs1 = U0getchar();    // read character
  U0putchar(input);     // echo character
}

void U0init(unsigned long U0baud)
{
//  Students are responsible for understanding
//  this initialization code for the ATmega2560 USART0
//  and will be expected to be able to intialize
//  the USART in differrent modes.
//
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
//
// Read USART0 RDA status bit and return non-zero true if set
//
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA; // Check the RDA status bit, and return True if the bit is set, return False if the bit is clear
}
//
// Read input character from USART0 input buffer
//
unsigned char U0getchar()
{
   return *myUDR0; // Return the character which has been received by the UART.
}
//
// Wait for USART0 (myUCSR0A) TBE to be set then write character to
// transmit buffer
//
void U0putchar(unsigned char U0pdata)
{  
  while (!(*myUCSR0A & TBE)){} //Wait until the serial port TBE status bit is high/TRUE, then take the character U0pdata and
  *myUDR0 = U0pdata; //write the character to the transmit buffer
}