//Computer Engineering 301:   Final Project
//File:                       Main Driver
//Authors:                    Mason Haines, Autsin Jarolimek, Samuel Mouradian
//Date of Last Revision:      05.03.2024
//Latest Edit Note:           RTC Finalized


//pins in use 
// A14 yellow, A12 green , blue,  red
//A15 on button, A0 stepper, A1 stepper 

// HEADERS
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include "DHT.h"
#include "DHT_U.h"


// Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.


#define DHTPIN 6  
#define DHTTYPE DHT11 
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

// pointers to I/O registers for port K
volatile unsigned char* port_K = (unsigned char*) 0x108; 
volatile unsigned char* ddr_K  = (unsigned char*) 0x107; 
volatile unsigned char* pin_K  = (unsigned char*) 0x106; 

// pointers to I/O registers for port E
volatile unsigned char* port_E = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_E  = (unsigned char*) 0x2D; 
volatile unsigned char* pin_E  = (unsigned char*) 0x2C; 
// #define PK6 (1 << 4) // PK6 pin A14
// #define PK5 (1 << 5) // PK5 pin A13

unsigned int timer_running; 
unsigned int currentTicks; 
const char *currentState = "DISABLED"; // Default state is DISABLED

DHT dht(DHTPIN, DHTTYPE);


// LCD pins <--> Arduino pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// STEPPER MOTOR VARIABLES
const int stepsPerRevolution = 2038;
// Stepper myStepper = Stepper(stepsPerRevolution, needs four pins in analog );



// SETUP
void setup() {
  U0init(9600); // Initialize Serial Port
  adc_init();   // Setup ADC
  setup_timer_regs();
  lcd.begin(16, 2); // set up number of columns and rows
  dht.begin();
  
  unsigned char value = 0b01111000; // Set DDR register to 0110 0000
  // Set PK7 as input with pull-up resistor enabled
  *ddr_K &= ~(1 << 7);  // Set bit 7 of DDRK to 0 for input pin A15
  *port_K |= (1 << 7);  // Set bit 7 of PORTK to 1 to enable pull-up resistor

  // Set PK6 and PK5 as outputs and turn off the LEDs initially
  *ddr_K |= value;  // Set bit 6 and bit 5 of DDRK to 1 for output
  *port_K |= (1 << 6); // set bit 6 of PORTK to turn on the yellow LED initially pin A14
  *port_K |= ~(1 << 5); // clear bit 5 of PORTK to turn off the green LED initially pin A13

}



// LOOP
void loop(){
  
  // // Read and print the state of each pin in PORTK
  // for (int pin = PK0; pin <= PK7; pin++) {
  //   int state = (PORTK >> pin) & 0x01; // Read the state of the pin in PORTK
  //   Serial.print("Pin PK");
  //   Serial.print(pin);
  //   Serial.print(" state: ");
  //   Serial.println(state);
  // }
  
  // // Add delay if necessary
  // delay(1000);
  // readAndPrint(cs1);
  // RTCtime();

// // Read ADC value from different channels to toggle stepper motor
//   int adcValue0 = adc_read(0); // Button 1
//   int adcValue1 = adc_read(1); // Button 2

// // Check if button 1 is pressed
//   if (adcValue0 > 800) {         
//     moveStepper(100);            // Move clockwise by 100 steps
//     while (adc_read(0) > 800) {} // Wait until button 1 is released
//     my_delay(100);
//   }

// // Check if button 2 is pressed
//   if (adcValue1 > 800) {         
//     moveStepper(-100);           // Move counterclockwise by 100 steps
//     while (adc_read(1) > 800) {} // Wait until button 2 is released
//     my_delay(100);
//   }

  

  
  int temperature = dht.readTemperature(true);
  int humidity = dht.readHumidity();

   // Check if readings are valid
  // if (isnan(temperature) || isnan(humidity)) {
  //   lcd.setCursor(0, 0);
  //   lcd.print("Error reading DHT!");
  //   return;
  // }

  // Print current state at the top portion of the LCD
  lcd.setCursor(0, 0); // Set cursor to the beginning of the first row
  lcd.print(currentState); // Display the current state on the LCD

  // Replace these values with actual temperature and humidity readings
  // temp = 10;
  // humidity = 20;

  // Print to LCD only if PK5 or PK6 is set
  lcd.setCursor(9, 0); // Set cursor to the beginning of the second row
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("*C");
  lcd.setCursor(9, 1);
  lcd.print("H:");
  lcd.print(humidity); // Display temperature and humidity on the LCD
  lcd.clear(); // Clear the LCD

  

  // Check if PK6 is set (yellow LED is on)
  if (*port_K & (1 << 6)) {
    
    currentState = "DISABLED"; // Update currentState to DISABLED
  }

  // Check if PK6 is cleared (yellow LED is off)
  if (!(*port_K & (1 << 6))) {
    
    currentState = "IDLE"; // Update currentState to IDLE
  }


}




// ISR to handle button press
ISR(TIMER1_OVF_vect)
{
  
  // Check if the button is pressed (PK7 is low)
  if (!(*pin_K & (1 << 7))) {
    // lcd.clear();
    // Toggle the LED only if it was previously off
    if (!(*port_K & (1 << 5))) {
      
      *port_K |= (1 << 5); // Turn on the green LED PK5
      *port_K &= ~(1 << 6); // Turn off the yellow LED PK6
      // currentState = "IDLE"; // Update currentState to IDLE
    } else {
      
      *port_K &= ~(1 << 5); // Turn off the green LED PK5
      *port_K |= (1 << 6); // Turn on the yellow LED PK6
      // currentState = "DISABLED"; // Update currentState to DISABLED
    }
    // Wait for the button to be released
    while (!(*pin_K & (1 << 7)));
  }
}

// Timer setup function
void setup_timer_regs()
{
  // Setup Timer1 for normal mode with overflow interrupt enabled
  TCCR1A = 0x00; // Normal mode
  TCCR1B = 0x01; // No prescaler, start the timer
  TCCR1C = 0x00; // Not used in normal mode

  // Enable Timer1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);
}



// // STEPPER MOTOR FUNCTIONS
// void moveStepper(int steps){
//   myStepper.setSpeed(5);
//   myStepper.step(steps);
// }

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
  for (int i = 0; i < 23; i++){
    U0putchar(time[i]);
  }
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

void readAndPrint(unsigned char input){
  unsigned char cs1;
  while (U0kbhit()==0){}; // wait for RDA = true
  cs1 = U0getchar();      // read character
  U0putchar(input);       // echo character
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