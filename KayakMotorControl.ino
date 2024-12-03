/*
  Kayak Motor Control door Benno Knaap
  Board Arduino Mega 2560 Pro
  Port /dev/cu.wchusbserial14640

  Scope 1.0V, 200ms, Trigger opgaand 2.1.V
   PWM. 1.0V, 1 ms, Trigger opgaand 850mV

*/
#include <LiquidCrystal_I2C.h>
/* 
I2C Input Output Pins 
Arduino Port Pin 20 : SDA : Physical Pin 44 : INT1
Arduino Port Pin 21 : SCL : Physical Pin 43 : INT0
*/
// Aantal RC ontvanger kanalen
#define RC_NUM_CHANNELS 6
// Setup Ontavnger kanalen PWM input Interrupt Gestuurd
#define RC_CH1 0 // Stick Links, Rechts
#define RC_CH2 1 // Stick Voorruit, Achterruit
#define RC_CH3 2 // t.b.d.
#define RC_CH4 3 // t.b.d.
#define RC_CH5 4 // t.b.d. No interrupt
#define RC_CH6 5 // t.b.d. No interrupt

// Setup aansluitingen voor de ontvanger op de arduino pins
#define RC_CH1_INPUT 2  // receiver CH1 op pin 2.    INT4
#define RC_CH2_INPUT 3  // receiver CH2 op pin 3     INT5
#define RC_CH3_INPUT 18 // receiver CH3 op pin 18    INT3
#define RC_CH4_INPUT 19 // receiver CH4 op pin 19    INT2
#define RC_CH5_INPUT 4  // receiver CH5 op pin 4   
#define RC_CH6_INPUT 5  // receiver CH6 op pin 5   



// Arduino Port Pin 56/A2 :ADC2 : Physical 95 for Loopcontrol analogpin werkt niet praktisch

// Setup data array verzameling om de pulsen op te slaan
uint16_t RC_VALUES[RC_NUM_CHANNELS];
uint32_t RC_START[RC_NUM_CHANNELS];
volatile uint16_t RC_SHARED[RC_NUM_CHANNELS];

// Setup Algemene schakeloutputs, spanning aan uit
#define Relay01 22 // Arduino Port Pin 22 : AD0 : Physical Pin 78
#define Relay02 23 // Arduino Port Pin 23 : AD1 : Physical Pin 77
#define Relay03 24 // Arduino Port Pin 24 : AD2 : Physical Pin 76
#define Relay04 25 // Arduino Port Pin 25 : AD3 : Physical Pin 75
// #define Relay05 26 // Arduino Port Pin 26 : AD4 : Physical Pin 74
// #define Relay06 27 // Arduino Port Pin 27 : AD5 : Physical Pin 73
// #define Relay07 28 // Arduino Port Pin 28 : AD6 : Physical Pin 72
#define PinLoopControl 26 // Arduino Port Pin 26 : AD4 : Physical Pin 74
#define PinOutRC3 27 // Arduino Port Pin 27 : AD5 : Physical Pin 73
#define PinOutRC4 28 // Arduino Port Pin 28 : AD6 : Physical Pin 72
#define Relay08 29 // Arduino Port Pin 29 : AD7 : Physical Pin 71

//      #define Potmeter 96 // Arduino Port Pin 55/A1 : ADC1 : Physical Pin 96 
//      LED_BUILTIN    Arduino Port Pin 13    : PB7 : Physical Pin 26 

// these constants won't change:
const int Potmeter = A0;  // Arduino Port Pin 54/A0 : ADC0 : Physical Pin 97. Potmeter
// PWM Outputs
const int PinRudderControl = 10;  // Geel. PWM Rudder`Control Left Write uitgangen om servo pulsbreedte aan te sturen  D2-D13, D44-D46, D52
const int PinSpeedControl = 11;  // Oranje PWM SpeedControl. 
const long AfkapWaardeChannel = 1000; // +/- 500 tot +/- 2500

LiquidCrystal_I2C lcdgreen(0x26,20,4);   // set the LCD address to 0x26 for a 16 chars and 4 line display
LiquidCrystal_I2C lcdblue(0x27,20,4);    // set the LCD address to 0x27 for a 16 chars and 4 line display

int deg_val = 0;
long width8 = 0;
long width16 = 0;
long width_min = 0;
long width_max = 1024;
long width8_min = 0;
long width8_max = 255;
int frequency = 50;
long sensorReading = 0;
int PotmeterReading = 0;
int LoopControl = 0;
int PulseRC5=0;
int PulseRC6=0;

// the setup function runs once when you press reset or power the board
void setup() {

   // initialialize the lcd
lcdgreen.init();
lcdblue.init();

  // Print a message to the LCD
lcdgreen.backlight();
lcdblue.backlight();
lcdgreen.setCursor(0,0);
lcdgreen.print("Kayak Motor Control1");
lcdgreen.setCursor(2,1);

lcdblue.setCursor(0,0);
lcdblue.print("Kayak Motor Control2");
lcdblue.setCursor(2,1);
// lcdblue.print(LED_BUILTIN);

// initialize digital pin LED_BUILTIN as an output.
// pinMode(LED_BUILTIN, OUTPUT);

pinMode(Relay01,OUTPUT);
pinMode(Relay02,OUTPUT);
pinMode(Relay03,OUTPUT);
pinMode(Relay04,OUTPUT);
// pinMode(Relay05,OUTPUT);
// pinMode(Relay06,OUTPUT);
// pinMode(Relay07,OUTPUT);
pinMode(PinLoopControl, INPUT); //Input voor Noodstop
pinMode(PinOutRC3, OUTPUT);
pinMode(PinOutRC4, OUTPUT);
pinMode(Relay08,OUTPUT);

// pinMode(Potmeter, INPUT);
// pinMode(analogOut, OUTPUT);
pinMode(RC_CH1_INPUT, INPUT);
pinMode(RC_CH2_INPUT, INPUT);
pinMode(RC_CH3_INPUT, INPUT);
pinMode(RC_CH4_INPUT, INPUT);
pinMode(RC_CH5_INPUT, INPUT);
pinMode(RC_CH6_INPUT, INPUT);

// Toewijzen interrupts
attachInterrupt(digitalPinToInterrupt(RC_CH1_INPUT), READ_RC1, CHANGE);
attachInterrupt(digitalPinToInterrupt(RC_CH2_INPUT), READ_RC2, CHANGE);
attachInterrupt(digitalPinToInterrupt(RC_CH3_INPUT), READ_RC3, CHANGE);
attachInterrupt(digitalPinToInterrupt(RC_CH4_INPUT), READ_RC4, CHANGE);
}

// the loop function runs over and over again forever
void loop() {
//  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
digitalWrite(Relay01, LOW);
digitalWrite(PinOutRC3, LOW); // Turn default on low
digitalWrite(PinOutRC4, LOW); // Turn default on low

lcdgreen.setCursor(12,0);
LoopControl = digitalRead(PinLoopControl);
lcdgreen.print( LoopControl);

  while (LoopControl == LOW) {

    digitalWrite(Relay01, HIGH);
    digitalWrite(Relay02, HIGH);
    digitalWrite(Relay03, HIGH);
    digitalWrite(Relay04, HIGH);
   // digitalWrite(Relay05, HIGH);
   // digitalWrite(Relay06, HIGH);
   // digitalWrite(Relay07, HIGH);
    digitalWrite(Relay08, HIGH);
    int sensorReading = analogRead(Potmeter);
    width8 = map(sensorReading,12, 1011, 0, 255);
    width16 = map(sensorReading,12,1011, 0, 1024);
    analogWrite(PinRudderControl, width8);
    analogWrite(PinSpeedControl, 128);
    PulseRC5 = pulseIn(RC_CH5_INPUT, HIGH);
    PulseRC6 = pulseIn(RC_CH6_INPUT, HIGH);
    
    rc_read_values();
    if (RC_VALUES[RC_CH3] >= AfkapWaardeChannel) {
      digitalWrite(PinOutRC3, HIGH);
    }
    else {
      digitalWrite(PinOutRC3, LOW);
    } 
    if (RC_VALUES[RC_CH4] >= AfkapWaardeChannel) {
      digitalWrite(PinOutRC4, HIGH);
    }
    else {
      digitalWrite(PinOutRC4, LOW);
    }
    // print de waarden naar de seriele poort in een format
    // Serial.print( RC_VALUES[RC_CH1]); Serial.print(",");
    // Serial.println( RC_VALUES[RC_CH2]); //Print line voor de laatste...
    
    lcdgreen.setCursor(0,1);
    lcdgreen.print("Potmeter:");
    lcdgreen.print(sensorReading);
    lcdgreen.print("      ");
    lcdgreen.setCursor(0,2);
    lcdgreen.print("  width8:");
    lcdgreen.print(width8);
    lcdgreen.print("      ");
    lcdgreen.setCursor(0,3);
    lcdgreen.print(" width16:");
    lcdgreen.print(width16);
    lcdgreen.print("      ");

    // Blauwe LCD voor RC ontvanger waarden
    lcdblue.setCursor(0,1);            //  (X,Y)
    lcdblue.print("CH1:      ");
    lcdblue.setCursor(4,1); 
    lcdblue.print(RC_VALUES[RC_CH1]);
    lcdblue.setCursor(0,2);            //  (X,Y)
    lcdblue.print("CH2:      ");
    lcdblue.setCursor(4,2);            //  (X,Y)
    lcdblue.print(RC_VALUES[RC_CH2]);
    lcdblue.setCursor(0,3);            //  (X,Y)
    lcdblue.print("CH3:      ");
    lcdblue.setCursor(4,3);            //  (X,Y)
    lcdblue.print(RC_VALUES[RC_CH3]);
    lcdblue.setCursor(10,1);           //  (X,Y)
    lcdblue.print("CH4:      ");
    lcdblue.setCursor(14,1);
    lcdblue.print(RC_VALUES[RC_CH4]); 
    lcdblue.setCursor(10,2);            //  (X,Y)
    lcdblue.print("CH5:      ");
    lcdblue.setCursor(14,2);   
    lcdblue.print(PulseRC5);
    lcdblue.setCursor(10,3);            //  (X,Y)
    lcdblue.print("CH6:      ");
    lcdblue.setCursor(14,3); 
    lcdblue.print(PulseRC6);
   

    // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(Relay01, LOW);
    digitalWrite(Relay02, LOW);
    digitalWrite(Relay03, LOW);
    digitalWrite(Relay04, LOW);
    // digitalWrite(Relay05, LOW);
    // digitalWrite(Relay06, LOW);
    // digitalWrite(Relay07, LOW);
    digitalWrite(Relay08, LOW);

     delay(100);                      // wait for a second
     lcdgreen.setCursor(12,0);
      LoopControl = digitalRead(PinLoopControl);
    lcdgreen.print( LoopControl);

  }

}



// The interuptfuncties

void READ_RC1() {
  Read_Input(RC_CH1, RC_CH1_INPUT);
}
void READ_RC2() {
  Read_Input(RC_CH2, RC_CH2_INPUT);
}
void READ_RC3() {
  Read_Input(RC_CH3, RC_CH3_INPUT);
}
void READ_RC4() {
  Read_Input(RC_CH4, RC_CH4_INPUT);
}
void READ_RC5() {
  Read_Input(RC_CH5, RC_CH5_INPUT);
}
void READ_RC6() {
  Read_Input(RC_CH6, RC_CH6_INPUT);
}

// lees de puls en slaat dat op
void Read_Input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    RC_START[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - RC_START[channel]);
    RC_SHARED[channel] = rc_compare;
  }
}
//Berekend the tijd van de puls en slaat dat op
void rc_read_values() {
  noInterrupts();
  memcpy(RC_VALUES, (const void *)RC_SHARED, sizeof(RC_SHARED));
  interrupts();
}
