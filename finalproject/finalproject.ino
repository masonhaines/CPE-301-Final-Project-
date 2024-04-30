//Computer Engineering 301:   Final Project
//File:                       Main Driver
//Authors:                    Mason Haines, Autsin Jarolimek, Samuel Mouradian
//Version:                    2.0
//Date of Last Revision:      04.30.2024
//Latest Edit Note:           Implemented Stepper Motor Code, RTC, and Delay Function


// HEADERS
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

// DEFINITIONS
#define RDA 0x80
#define TBE 0x20  

// UART POINTERS
volatile unsigned char *myUCSR0A = (unsigned char *) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *)  0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *) 0x00C6;

// ADC POINTERS
volatile unsigned char* my_ADCSRA  = (unsigned char*) 0x7A;
volatile unsigned char* my_ADCSRB  = (unsigned char*) 0x7B;
volatile unsigned char* my_ADMUX   = (unsigned char*) 0x7C;
volatile unsigned int* my_ADC_DATA = (unsigned int*)  0x78;

// TIMER POINTERS
volatile unsigned char *myTIFR1  = (unsigned char *) 0x36;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;

// STEPPER MOTOR VARIABLES
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);



// SETUP
void setup() {
  adc_init();   // Setup ADC
  U0init(9600); // Initialize Serial Port
}



// LOOP
void loop(){
  readAndPrint(cs1);
  RTCtime();

// Read ADC value from different channels
  int adcValue0 = adc_read(0); // Button 1
  int adcValue1 = adc_read(1); // Button 2

// Check if button 1 is pressed
  if (adcValue0 > 800) {         // Assuming a threshold value for button press
    moveStepper(100);            // Move clockwise by 100 steps
    while (adc_read(0) > 800) {} // Wait until button 1 is released
    my_delay(100);
  }

// Check if button 2 is pressed
  if (adcValue1 > 800) {         // Assuming a threshold value for button press
    moveStepper(-100);           // Move counterclockwise by 100 steps
    while (adc_read(1) > 800) {} // Wait until button 2 is released
    my_delay(100);
  }


}







void readAndPrint(unsigned char input){
  unsigned char cs1;
  while (U0kbhit()==0){}; // wait for RDA = true
  cs1 = U0getchar();      // read character
  U0putchar(input);       // echo character
}

void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return *myUCSR0A & RDA;
}

unsigned char U0getchar(){
   return *myUDR0;
}

void U0putchar(unsigned char U0pdata){
  while (!(*myUCSR0A & TBE)){}
  *myUDR0 = U0pdata;
}



// STEPPER MOTOR FUNCTIONS
void moveStepper(int steps){
  myStepper.setSpeed(5);
  myStepper.step(steps);
}


// ADC REGISTER SETUP FUNCTION
void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}


// ADC READ FUNCITON
unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}


// DELAY FUNCTION
void my_delay(unsigned int freq){
  double period = 1.0/double(freq);               //Calculate period
  double half_period = period/ 2.0f;              //50% duty cycle
  double clk_period = 0.0000000625;               //Clock period def
  unsigned int ticks = half_period / clk_period;  //Calculate ticks

  *myTCCR1B &= 0xF8;                              //Stop the timer
  *myTCNT1 = (unsigned int) (65536 - ticks);      //Set the counts
  *myTCCR1B |= 0b00000001;                        //Start the timer

  while((*myTIFR1 & 0x01)==0);                    //Wait for overflow 
  *myTCCR1B &= 0xF8;                              //Stop the timer
  *myTIFR1 |= 0x01;                               //Reset TOV
}


// REAL TIME CLOCK FUNCTION
void RTCtime(){
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char time[24] = {
    'a',
    't',
    ' ',
    hour / 10 + '0',
    hour % 10 + '0',
    ':',
    minute / 10 + '0',
    minute % 10 + '0',
    ':',
    second / 10 + '0',
    second % 10 + '0',
    'o',
    'n',
    month / 10 + '0',
    month % 10 + '0',
    '/',
    day / 10 + '0',
    day % 10 + '0',
    '/',
    (year / 1000) + '0',
    (year % 1000 / 100) + '0',
    (year % 100 / 10) + '0',
    (year % 10) + '0',
  };
  for (int i = 0; i < 23; i++)
  {
    U0putchar(time[i]);
  }
}




// volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
// volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
// volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
// volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
// volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
// volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
// volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
// volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
// volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
// volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
// volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
// volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
// volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
// volatile unsigned char *myTIFR1  = (unsigned char *) 0x36;

// // Define pointers to I/O registers for port K
// volatile unsigned char* port_k = (unsigned char*) 0x108; 
// volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
// volatile unsigned char* pin_k  = (unsigned char*) 0x106; 

// // Define pointers to I/O registers for port E
// volatile unsigned char* port_E = (unsigned char*) 0x2E; 
// volatile unsigned char* ddr_E  = (unsigned char*) 0x2D; 
// volatile unsigned char* pin_E  = (unsigned char*) 0x2C; 

// unsigned int timer_running; 
// unsigned int currentTicks; 


// void setup() {
//   // Set PK7 as input with pull-up resistor enabled
//   *ddr_k &= ~(1 << 7);  // Set bit 7 of DDRK to 0 for input
//   *port_k |= (1 << 7);  // Set bit 7 of PORTK to 1 to enable pull-up resistor

//   // Set PE0 as output and turn off the LED initially
//   *ddr_E |= (1 << 0);   // Set bit 0 of DDRE to 1 for output
//   *port_E &= (1 << 0); // Clear bit 0 of PORTE to turn off the LED initially

//   // Initialize Timer
//   setup_timer_regs();
// }

// void loop() {
//   // No need for code here, ISR handles the button press
// }

// // ISR to handle button press
// ISR(TIMER1_OVF_vect)
// {
//   // Check if the button is pressed (PK7 is low)
//   if ((*pin_k & (1 << 7))) {
//     // Toggle the LED only if it was previously off
//     if (!(*port_E & (1 << 0))) {
//       *port_E |= (1 << 0); // Turn on the LED
//     } else {
//       *port_E &= ~(1 << 0); // Turn off the LED
//     }
//     // Wait for the button to be released
//     while ((*pin_k & (1 << 7)));
//   }
// }

// // Timer setup function
// void setup_timer_regs()
// {
//   // Setup Timer1 for normal mode with overflow interrupt enabled
//   TCCR1A = 0x00; // Normal mode
//   TCCR1B = 0x01; // No prescaler, start the timer
//   TCCR1C = 0x00; // Not used in normal mode

//   // Enable Timer1 overflow interrupt
//   TIMSK1 |= (1 << TOIE1);
// }