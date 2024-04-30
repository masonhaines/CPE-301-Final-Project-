//Final Project - Computer Engineering 301
//Fan Motor Implementation
//Written by: Samuel Mouradian // editted by Mason Haines

// volatile unsigned char* port_E = (unsigned char*) 0x2E; // output register 
// volatile unsigned char* ddr_E = (unsigned char*) 0x2D; // data direction register
// volatile unsigned char* pin_E = (unsigned char*) 0x2C; // input register 

// volatile unsigned char* port_G = (unsigned char*) 0x34; // output register 
// volatile unsigned char* ddr_G = (unsigned char*) 0x33; // data direction register
// volatile unsigned char* pin_G = (unsigned char*) 0x32; // input register 

volatile unsigned char* port_D = (unsigned char*) 0x2B; // output register 
volatile unsigned char* ddr_D = (unsigned char*) 0x2A; // data direction register
volatile unsigned char* pin_D = (unsigned char*) 0x29; // input register 

void setup() {
  
  Serial.begin(9600);
  //Set PD0 = pin 21 & PD1 = pin 20 & PD2 = pin 19
  *ddr_D |= 0x07; // Set first registers in ddr to output -> communication pins on arduino -> is an OR set 0000_0111
}

void loop() {
  toggleMotor_ON_OFF();
  delay(1000); // Adjust the delay time as needed
}

void toggleMotor_ON_OFF() { // this is the beta version will need tempps to know ehen to turn on, so will need to be passed arguments 
  // Turn the fan on (clockwise direction) at full speed

  // Set PD0 to HIGH to set the motor direction (clockwise)
  *port_D |= 0x01; // Set PD0

  // Clear PD1 to ensure no conflicting direction setting
  *port_D &= ~0x02; // Clear PD1

  // Set PD2 to enable the motor at full speed
  *port_D |= 0x04; // Set PD2

  delay(2000); // Run the fan for 5 seconds

  // Turn the fan off

  // Clear PD0 to turn off the motor
  *port_D &= ~0x01; // Clear PD0

  // Clear PD1 to ensure no conflicting direction setting
  *port_D &= ~0x02; // Clear PD1

  // Clear PD2 to disable the motor
  *port_D &= ~0x04; // Clear PD2

  delay(2000); // Wait for 2 seconds before repeating the cycle
}