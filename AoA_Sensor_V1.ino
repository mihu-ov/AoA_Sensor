/*
AoA Sensor based on AS5045 chip
Uses timer1 to measure pulses on pin 8 on 168/328
Pulse measurement adapted from
//    FILE: PulseWidthMeter.pde
//  AUTHOR: Rob Tillaart
//    DATE: 2012-apr-01
//
//    LINK: http://arduino.cc/forum/index.php?action=post;topic=96971.0
//
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Narcoleptic.h>
#include <stdlib.h>

volatile unsigned int count = 0;
unsigned int pulseCounter = 0;

const int inputCapturePin = 8; // input pin 
const int AMS5045_VCC = 7;
//const int ledPin = 13; 
const int bt_mode_pin = 11;
const int bt_reset_pin = 3;
const int wakeup_delay = 20;
const int sampling_delay = 10;
const int serial_delay = 10;
const unsigned int baudrate = 57600; //define baudrate
//const int SamplePeriod = 200 - wakeup_delay - sampling_delay - serial_delay; // the sample period in milliseconds
const int SamplePeriod = 160; // the sample period in milliseconds
const float alpha0 = 2600; // sensor position for alpha = 0 indication
const float d_alpha = 170; // +/-15°, 15° * 11.37 = 170
const float vario_max = 5; // XCSoar vario gauge maximum value 5m/s
float angular_position;
char ang_pos_str[50];
char LXWP0_char[51];
char cs_char[10];
volatile unsigned int ton; // on time 16 bit value
volatile unsigned int tges; // total cycle ton + toff 16 bit value

// Calculates the checksum for a given string
// returns as integer
// From Tim Zaman http://www.timzaman.com/?p=140&lang=en

int getCheckSum(char *string) {
  int i;
  int XOR;
  int c;
  // Calculate checksum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < strlen(string); i++) {
    c = (unsigned char)string[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  return XOR;
}


void setup()
{
  Serial.begin(baudrate);
  Serial.println("AoA_Sensor V1");
//  pinMode(ledPin, OUTPUT);
  digitalWrite(bt_mode_pin, LOW); // Mode = Low -> Binding Mode, High -> AT Command Mode
  digitalWrite(bt_reset_pin, HIGH); // Reset when LOW
  pinMode(AMS5045_VCC, OUTPUT); 
  pinMode(inputCapturePin, INPUT); // ICP pin (digital pin 8 on Arduino) as input
  TCCR1A = 0 ; // Normal counting mode
  TCCR1B = 0 ; // set prescale bits
}

void loop()
{
  digitalWrite(AMS5045_VCC, HIGH); //Power on AS5045 sensor chip
  // digitalWrite(ledPin, HIGH);
  delay(wakeup_delay);             //Delay until AS5045 is ready
 
  // measure sensor ouput on time (1-4096µs)
  // reset Timer1 registers and the counters (timer must be stopped before reset of counters!.
  TCCR1A = 0;
  TCCR1B = 0; 
  count = 0;
  TCNT1 = 0;
  TIMSK1 = (1 << TOIE1);                      // enable Timer1 overflow interrupt:
  while ((PINB & B00000001) == B00000001);    // wait for low
  while ((PINB & B00000001) == B00000000);    // wait for HIGH
  TCCR1B |= (1 << CS10);                      // Set CS10 bit so timer runs at clock speed: 8 MHz
  while ((PINB & B00000001) == B00000001);    // wait for low
  TCCR1B = 0;                                 // stop counting
  ton = TCNT1; 

  // measure sensor total cycle time
  // reset Timer1 registers and the counters (timer must be stopped before reset of counters!.  
  TCCR1A = 0;
  TCCR1B = 0; 
  count = 0;
  TCNT1 = 0;
  TIMSK1 = (1 << TOIE1);                      // enable Timer1 overflow interrupt:
  while ((PINB & B00000001) == B00000001);    // wait for low
  while ((PINB & B00000001) == B00000000);    // wait for HIGH
  TCCR1B |= (1 << CS10);                      // Set CS10 bit so timer runs at clock speed: 8 MHz
  while ((PINB & B00000001) == B00000001);    // wait for low
  while ((PINB & B00000001) == B00000000);    // wait for HIGH 
  TCCR1B = 0;                                 // stop counting
  tges = TCNT1;   
 

  // digitalWrite(ledPin, LOW);
  digitalWrite(AMS5045_VCC, LOW); //Power off sensor
  // angular position, 0 = 0deg , 4095 = 359.91deg
  //  angular_position = ((((float(ton) * 4097.0 / float(tges))- 1.0)));
  //  Serial.println(angular_position);
  angular_position = ((((float(ton) * 4097.0 / float(tges))- 1.0))-alpha0)*(vario_max/d_alpha);
  dtostrf(angular_position,6,2,ang_pos_str);
  LXWP0_char[0] = '\0';
  strcat(LXWP0_char, "$LXWP0,,,,");
  strcat(LXWP0_char, ang_pos_str);
  strcat(LXWP0_char, ",,,,,,,*");
  itoa(getCheckSum(LXWP0_char), cs_char, HEX);
  strcat(LXWP0_char, cs_char);
 // Serial.println(ton);
 // Serial.println(tges);
  Serial.println(LXWP0_char); // angular position, 0 = 0deg , 4095 = 359.91deg
  delay(serial_delay);
  Narcoleptic.delay(SamplePeriod);  
  
}


