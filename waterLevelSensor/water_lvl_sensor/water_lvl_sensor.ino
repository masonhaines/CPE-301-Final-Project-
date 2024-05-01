//Austin Jarolimek - Final Project
// Check the water level sensor

//Define ADC Register Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];

void setup()
{
  Serial.begin(9600);
  adc_init();
}

void loop()
{
  checkADCValue(adc_id, HistoryValue);
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= B10000000;
  *my_ADCSRA &= B11110111;
  *my_ADCSRA &= B11011111;
  
  // setup the B register
  *my_ADCSRB &= B11111000;

  // setup the MUX Register
  *my_ADMUX |= (1<<REFS0);
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  int channel_selector;
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= B11100000;

  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= B11110111;

  //Assign correct channel using MUX 5:0
  if (adc_channel_num < 8) {
    *my_ADMUX |= adc_channel_num;
  }
  else if ((adc_channel_num > 7) && (adc_channel_num < 16)) {
     channel_selector = (adc_channel_num - 8);
     *my_ADCSRB |= B00001000;
     *my_ADMUX |= channel_selector;
  }

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= B01000000;
  
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return (*my_ADC_DATA & 0x03FF);
}

// Function to print ADC value to Serial
void printADCValue(int adc_id, int value) {
  sprintf(printBuffer, "ADC%d level is %d\n", adc_id, value);
  Serial.print(printBuffer);
}

// Function to check ADC value and print if significant change is detected
void checkADCValue(int adc_id, int historyValue) {
  int value = adc_read(adc_id); // Get current ADC value

  // Check for significant change in ADC value
  if (((historyValue >= value) && ((historyValue - value) > 10)) ||
      ((historyValue < value) && ((value - historyValue) > 10))) {
    printADCValue(adc_id, value); // Print ADC value
    HistoryValue = value; // Update history value
  }
}