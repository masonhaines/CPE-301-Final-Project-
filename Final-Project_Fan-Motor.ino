//Final Project - Computer Engineering 301
//Fan Motor Implementation
//Written by: Samuel Mouradian

//DC Fan motor
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;
volatile unsigned char* pin_g  = (unsigned char*) 0x32;

//Fan States
#define fanOn 0x20
#define fanOff 0x00