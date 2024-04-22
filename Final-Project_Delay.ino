//Final Project - Delay Function
//Written by: Samuel Mouradian - 04.22.2024

//TIMER POINTERS
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1  = (unsigned char *) 0x36;

void setup(){}

void loop() {}

void my_delay(unsigned int freq){
  doub  le period = 1.0/double(freq);  //Calculate period
  double half_period = period/ 2.0f;  //50% duty cycle
  double clk_period = 0.0000000625;  //Clock period def
  unsigned int ticks = half_period / clk_period;  //Calculate ticks
  *myTCCR1B &= 0xF8;  //Stop the timer
  *myTCNT1 = (unsigned int) (65536 - ticks);  //Set the counts
  *myTCCR1B |= 0b00000001;  //Start the timer
  while((*myTIFR1 & 0x01)==0);  //Wait for overflow 
  *myTCCR1B &= 0xF8;  //Stop the timer
  *myTIFR1 |= 0x01;  //Reset TOV
}