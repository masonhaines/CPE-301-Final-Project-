// Includes the Arduino Stepper Library
#include <Stepper.h>

// // Defines the number of steps per rotation
// const int stepsPerRevolution = 2038;

// // Creates an instance of stepper class
// // Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
// Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

// int prevButtonState1 = LOW;
// int prevButtonState2 = LOW;

// void setup() {
//   pinMode(2, INPUT); // Button pin 1
//   pinMode(3, INPUT); // Button pin 2
//   pinMode(4, OUTPUT);
//   pinMode(5, OUTPUT);
//   // Nothing to do (Stepper Library sets pins as outputs)
// }

// void loop() {
//   // Read the state of the first button
//   int buttonState1 = digitalRead(2); // Button pin 1

//   // Check if the first button is pressed and was not previously pressed
//   if (buttonState1 == HIGH && prevButtonState1 == LOW) {
//     digitalWrite(4, HIGH); // Turn on LED connected to pin 4

//     // Move the stepper motor clockwise by an incremental amount
//     myStepper.setSpeed(5);
//     myStepper.step(100); // Change this value to adjust the amount of steps
//     delay(500); // Delay for smoother operation

//     digitalWrite(4, LOW); // Turn off LED connected to pin 4
//   }

//   // Read the state of the second button
//   int buttonState2 = digitalRead(3); // Button pin 2

//   // Check if the second button is pressed and was not previously pressed
//   if (buttonState2 == HIGH && prevButtonState2 == LOW) {
//     digitalWrite(5, HIGH); // Turn on LED connected to pin 5

//     // Move the stepper motor counter-clockwise by an incremental amount
//     myStepper.setSpeed(5);
//     myStepper.step(-100); // Change this value to adjust the amount of steps
//     delay(500); // Delay for smoother operation

//     digitalWrite(5, LOW); // Turn off LED connected to pin 5
//   }

//   // Update the previous state of the buttons
//   prevButtonState1 = buttonState1;
//   prevButtonState2 = buttonState2;
// }

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// const int stepsPerRevolution = 2038;
// Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

// void setup() {
//   // Setup ADC
//   adc_init();
// }

// void loop() {
//   // Move stepper motor based on ADC value from different channels
//   stepperMotorAdjust(0, 100); // Adjust based on ADC channel 0
//   stepperMotorAdjust(1, -100); // Adjust based on ADC channel 1
//   // Add more calls to adjust based on other ADC channels as needed
// }

// void stepperMotorAdjust(int adc_channel_num, int steps) {
//   // Read ADC value
//   unsigned int adcValue = adc_read(adc_channel_num);
  
//   // Determine direction based on ADC value
//   int direction;
//   if (adcValue > 512) {
//     direction = 1;
//   } else {
//     direction = -1;
//   }
  
//   // Move stepper motor accordingly
//   myStepper.setSpeed(5);
//   myStepper.step(direction * steps);
// }

const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  // Setup ADC
  adc_init();
}

void loop() {
  // Read ADC value from different channels
  int adcValue0 = adc_read(0); // Button 1
  int adcValue1 = adc_read(1); // Button 2

  // Check if button 1 is pressed
  if (adcValue0 > 800) { // Assuming a threshold value for button press
    moveStepper(100); // Move clockwise by 100 steps
    while (adc_read(0) > 800) {} // Wait until button 1 is released
  }

  // Check if button 2 is pressed
  if (adcValue1 > 800) { // Assuming a threshold value for button press
    moveStepper(-100); // Move counterclockwise by 100 steps
    while (adc_read(1) > 800) {} // Wait until button 2 is released
  }
}

void moveStepper(int steps) {
  myStepper.setSpeed(5);
  myStepper.step(steps);
}








void adc_init()
{
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
unsigned int adc_read(unsigned char adc_channel_num)
{
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































// #include <LiquidCrystal.h>


// #define RDA 0x80
// #define TBE 0x20  

// volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
// volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
// volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
// volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
// volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;



// void setup() {
//   // put your setup code here, to run once:
//   // initialize the serial port on USART0:
//  U0init(9600);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   unsigned char cs1;
//   while (U0kbhit()==0){}; // wait for RDA = true
//   cs1 = U0getchar();    // read character

//   readAndPrint(cs1);
// }

// void readAndPrint(unsigned char input) {
//   // unsigned char cs1;
//   // while (U0kbhit()==0){}; // wait for RDA = true
//   // cs1 = U0getchar();    // read character
//   U0putchar(input);     // echo character
// }

// void U0init(unsigned long U0baud)
// {
// //  Students are responsible for understanding
// //  this initialization code for the ATmega2560 USART0
// //  and will be expected to be able to intialize
// //  the USART in differrent modes.
// //
//  unsigned long FCPU = 16000000;
//  unsigned int tbaud;
//  tbaud = (FCPU / 16 / U0baud - 1);
//  // Same as (FCPU / (16 * U0baud)) - 1;
//  *myUCSR0A = 0x20;
//  *myUCSR0B = 0x18;
//  *myUCSR0C = 0x06;
//  *myUBRR0  = tbaud;
// }
// //
// // Read USART0 RDA status bit and return non-zero true if set
// //
// unsigned char U0kbhit()
// {
//   return *myUCSR0A & RDA; // Check the RDA status bit, and return True if the bit is set, return False if the bit is clear
// }
// //
// // Read input character from USART0 input buffer
// //
// unsigned char U0getchar()
// {
//    return *myUDR0; // Return the character which has been received by the UART.
// }
// //
// // Wait for USART0 (myUCSR0A) TBE to be set then write character to
// // transmit buffer
// //
// void U0putchar(unsigned char U0pdata)
// {  
//   while (!(*myUCSR0A & TBE)){} //Wait until the serial port TBE status bit is high/TRUE, then take the character U0pdata and
//   *myUDR0 = U0pdata; //write the character to the transmit buffer
// }