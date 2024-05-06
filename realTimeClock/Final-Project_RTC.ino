//Final Project:     Computer Engineering 301
//File Description:  RTC Implementation
//Written by:        Samuel Mouradian

#include <RTClib.h>
#define TBE 0x20
RTC_DS1307 rtc;

volatile unsigned char *myUCSR0A = (unsigned char *) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *)  0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *) 0x00C6;

enum State { DISABLED, RUNNING, IDLE, ERROR };
State currentState = DISABLED;
State tempState;


void setup() {
  Serial.begin(9600);
  
  if(!rtc.begin()){
    U0putstring("Couldn't find RTC");
    while(1);
  }
  else{
    U0putstring("RTC Connected ");
  }
  if(!rtc.isrunning()){
    U0putstring("RTC is not running, setting time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop(){
  if(currentState != tempState){
    RTCtime();
    tempState = currentState;
  }
  else{
    currentState = tempState;
  }
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
    (year % 10) + '0',
  };
  for (int i = 0; i < 23; i++)
  {
    U0putchar(time[i]);
  }
  U0putstring('\n');
}

void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
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