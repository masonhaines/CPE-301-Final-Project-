//Computer Engineering 301:   Final Project
//File:                       Main Driver
//Authors:                    Mason Haines, Autsin Jarolimek, Samuel Mouradian
//Version:                    2.0
//Date of Last Revision:      05.1.2024
//Latest Edit Note:           Implemented Stepper Motor Code, RTC, and Delay Function

// HEADERS
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include "DHT.h"
#include "DHT_U.h"

#define DHTPIN 14  
#define DHTTYPE DHT11 
#define RDA 0x80
#define TBE 0x20  
RTC_DS1307 rtc;

enum State { DISABLED, RUNNING, IDLE, ERROR };
State currentState = DISABLED;
State tempState = ERROR;
const char *LCDState = "DISABLED"; // Default state is DISABLED

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
// pointers to I/O registers for port D
volatile unsigned char* port_D = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_D = (unsigned char*) 0x2A; 
volatile unsigned char* pin_D = (unsigned char*) 0x29; 
// pointers to I/O registers for port F
volatile unsigned char* port_F = (unsigned char*) 0x31; 
volatile unsigned char* ddr_F = (unsigned char*) 0x30; 
volatile unsigned char* pin_F = (unsigned char*) 0x2F; 

unsigned long lastTempUpdate = 0; // Variable to store the last time temperature was updated
bool initialTempFlag = false; // Flag to track if initial readings have been done
int temperature = 0; 
int humidity = 0; 

// DHT temp and humidty sensor object creation
DHT dht(DHTPIN, DHTTYPE);
int waterSens_id = 7; // Water sensor analog pin ID

// LCD pin setup 
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Stepper Motor initalization
const int stepsPerRevolution = 2038;
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 15, 17, 16, 18);

// SETUP
void setup() {
  U0init(9600); // Initialize Serial Port

  // Check if the RTC (Real-Time Clock) module is successfully initialized
  if(!rtc.begin()){ // If RTC initialization fails, print an error message and halt the program
    U0putstring("Couldn't find RTC");
    while(1);
  } else {
    // If RTC initialization succeeds, print a confirmation message
    U0putstring("RTC Connected ");
  }

  // Check if the RTC module is running
  if(!rtc.isrunning()){ // If the RTC module is not running, print a message indicating that the time needs to be set
      
    U0putstring("RTC is not running, setting time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  setup_timer_regs();
  lcd.begin(16, 2); // Initialize the LCD display with 16 columns and 2 rows
  dht.begin(); // Initialize the DHT (temperature and humidity sensor) module

  // Set up the GPIO pins for the fan motor
  *ddr_K |= (1 << 0); // Set pin 0 of port K as output for fan motor
  *ddr_K |= (1 << 1); // Set pin 1 of port K as output for fan motor
  *ddr_K |= (1 << 2); // Set pin 2 of port K as output for fan motor

  // Stepper motor for Vent
  *ddr_F &= ~(1 << 0); // Clear bit 1 of DDRF to configure it as input
  *ddr_F &= ~(1 << 1); // Clear bit 1 of DDRF to configure it as input
  
  // Set PK7 as input with pull-up resistor enabled
  *ddr_K &= ~(1 << 7);  // Set bit 7 of DDRK to 0 for input pin A15
  *port_K |= (1 << 7);  // Set bit 7 of PORTK to 1 to enable pull-up resistor

  // Set PK6 and PK5 as outputs and turn off the LEDs initially
  unsigned char value = 0b01111000; // Set DDR register to 0110 0000
  *ddr_K |= value;  // Set bit 6 and bit 5 of DDRK to 1 for output
  *port_K |= (1 << 6); // set bit 6 of PORTK to turn on the yellow LED initially pin A14
  
  adc_init();   // Setup ADC
}

// LOOP
void loop() {
  // Check if the current state is different from the temporary state
  if(currentState != tempState){ // If the states are different, update the real-time clock (RTC) time
    RTCtime();
    tempState = currentState; // Update the temporary state to match the current state
  } else {  // If the states are the same, update the current state to match the temporary state
    currentState = tempState;
  }

  // Check if initial readings have been done
  if (!initialTempFlag) {
    temperature = dht.readTemperature(true);
    humidity = dht.readHumidity();

    // Check if readings are valid
    if (temperature != -999 && humidity != -999) {
      initialTempFlag = true; // Set the flag to true once initial readings are done
    }
  } else {
    // Check if the system is in an idle state
    // Print current state at the top portion of the LCD
    lcd.setCursor(0, 0); // Set cursor to the beginning of the first row
    lcd.print(LCDState); // Display the current state on the LCD

    if (!DISABLED) {
      // Check if it's time to update the temperature
      unsigned long currentMillis = millis();
      if (currentMillis - lastTempUpdate >= 60000) {
        lastTempUpdate = currentMillis; // Update the last update time
        temperature = dht.readTemperature(true);
        humidity = dht.readHumidity();

        // Check if readings are valid
        if (isnan(temperature) || isnan(humidity)) {
          lcd.setCursor(0, 0);
          lcd.print("Error reading DHT!");
          return;
        }
      }
    }
  }

  uint8_t clearLights = ~(0x78); // 0x78 01111000, light reset
  uint8_t turnOffFan = ~(0x07); // Mask to turn off fan motor 

  switch (currentState) {
    case DISABLED:

      *port_K &= turnOffFan; // Mask for fan motor, turn off fan motor
      LCDState = "DISABLED"; // Update currentState to DISABLED
      if(!(*port_K & (1 << 6))) {*port_K &= clearLights;}
      *port_K |= (1 << 6); // Turn on the yellow LED PK6
      
      break;
    case RUNNING:

      LCDState = "RUNNING"; // Update currentState to RUNNING
      if(!(*port_K & (1 << 4))) {*port_K &= clearLights;}
      *port_K |= (1 << 4); // Turn on the blue LED PK4

      // temperature = dht.readTemperature(true); // FOR TESTING ---------------------------------
      // humidity = dht.readHumidity(); // FOR TESTING ---------------------------------

      printHumidTemp(temperature, humidity); 
      checkWaterLevel(waterSens_id); 
      
      // Start fan
      toggleMotor_ON();
      // If temp from DHT sensor is below threshold stop fan motor and return to IDLE state
      if (dht.readTemperature(true) < 74) {
        *port_D &= turnOffFan; // Mask for fan motor
        currentState = IDLE;
      }
      
      // Check if button 1 is pressed
      if (*pin_F & (1 << 0)) { 
        myStepper.setSpeed(5);
        myStepper.step(100);
        while (*pin_F & (1 << 0)) {} // Wait until button 1 is released
      }
      if (*pin_F & (1 << 1)) { 
        myStepper.setSpeed(5);
        myStepper.step(-100);
        while (*pin_F & (1 << 1)) {} // Wait until button 2 is released
      }
      break;
    case IDLE:

      LCDState = "IDLE"; // Update currentState to IDLE
      if(!(*port_K & (1 << 5))) {*port_K &= clearLights;}
      *port_K |= (1 << 5); // Turn on the green LED PK5
      *port_K &= turnOffFan; // Mask for fan motor, turn off fan motor
      
      // temperature = dht.readTemperature(true); // FOR TESTING ---------------------------------
      // humidity = dht.readHumidity(); // FOR TESTING ---------------------------------

      printHumidTemp(temperature, humidity);
      checkWaterLevel(waterSens_id); // this is for testing 

      // If temp from DHT sensor is above threshold go to IDLE state
      if (dht.readTemperature(true) > 75) {
        currentState = RUNNING;
      }

      // Check if button 1 is pressed
      if (*pin_F & (1 << 0)) { 
        myStepper.setSpeed(5);
        myStepper.step(100);
        while (*pin_F & (1 << 0)) {} // Wait until button 1 is released
      }
      if (*pin_F & (1 << 1)) { 
        myStepper.setSpeed(5);
        myStepper.step(-100);
        while (*pin_F & (1 << 1)) {} // Wait until button 2 is released
      }
      break;
    case ERROR:

      LCDState = "ERROR"; // Update currentState to ERROR // 0ygb r000
      if(!(*port_K & (1 << 3))) {*port_K &= clearLights;} // 0000 0000
      *port_K |= (1 << 3); // Turn on the Red LED PK3
      *port_K &= turnOffFan; // Mask for fan motor, turn off fan motor

      checkWaterLevel(waterSens_id); // this is for testing 
      lcd.setCursor(0, 1);
      lcd.print("H2o level low: ");
      break;
  }
}

void toggleMotor_ON() { 
  // Set PK0 to HIGH to set the motor direction (clockwise)
  *port_K |= 0x01; // Set P
  // Clear PK1 to ensure no conflicting direction setting
  *port_K &= ~0x02; // Clear P
  // Set PK2 to enable the motor at full speed
  *port_K |= 0x04; // Set P
}

void printHumidTemp(int temperature, int humidity) {
  lcd.setCursor(9, 0); // Set cursor to the beginning of the second row
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("*F");
  lcd.setCursor(9, 1);
  lcd.print("H:");
  lcd.print(humidity); // Display temperature and humidity on the LCD
  lcd.clear(); // Clear the LCD
}


// Function to check ADC value and print if significant change is detected
void checkWaterLevel(int waterSens_id) {
  int waterLevel = adc_read(waterSens_id); // Get current ADC waterLevel

  // Check if the ADC waterLevel is under 25
  if (waterLevel < 25) {
    // lcd.clear();
    currentState = ERROR; // Change state to ERROR
    return; // Exit the function
  }
  if (((currentState == ERROR) && (waterLevel > 25)) && (*pin_K & (1 << 7))) {
    lcd.clear();
    my_delay(2);
    currentState = IDLE;
  }
}


// ISR to handle button press
ISR(TIMER1_OVF_vect)
{
  // Check if the button is pressed (PK7 is low)
  if ((*pin_K & (1 << 7))) {
    lcd.clear();
    
    if (currentState == DISABLED) {
      currentState = IDLE; // If the current state is disabled, change it to idle
    } else {
      lcd.clear();
      currentState = DISABLED; // If it's anything other than disabled, change it to disabled
    }
    // Wait for the button to be released
    while ((*pin_K & (1 << 7)));
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


void RTCtime(){
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char time[26] = {
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
    ' ',
    'o',
    'n',
    ' ',
    month / 10 + '0',
    month % 10 + '0',
    '/',
    day / 10 + '0',
    day % 10 + '0',
    '/',
    (year / 1000) + '0',
    (year % 1000 / 100) + '0',
    (year % 100 / 10) + '0',
    (year % 10) + '0', ' '
  };
  for (int i = 0; i < 26; i++)
  {
    U0putchar(time[i]);
  }
   U0putchar('\n');
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

void U0putchar(unsigned char U0pdata){
  while (!(*myUCSR0A & TBE)){}
  *myUDR0 = U0pdata;
}

void U0putstring(char* U0pstring) {
  while(*U0pstring) {
      while(!(*myUCSR0A & TBE));
      *myUDR0 = *U0pstring;
      U0pstring++;
  }
  // Print a new line after printing the string
  while(!(*myUCSR0A & TBE));
  *myUDR0 = '\n';
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