//#include <Wire.h>
#include <math.h>
#include <FrequencyTimer2.h>
#include <WS2812Serial.h>
#include <TimerOne.h>
#include <Bounce2.h>
#include <i2c_t3.h>
//#include "SFE_HMC6343.h"
#include <Encoder.h>

Encoder knobLeft(14, 17);
Encoder knobRight(12, 11);
long positionLeft  = -999;
long positionRight = -999;


//SFE_HMC6343 compass; // Declare the sensor object

  uint8_t rawData[6];  //magnetometer buffer
  int first, sec, third, magState;

const uint8_t addr = 0x70; // HT16K33 default address
uint16_t displayBuffer[8];
const uint16_t lookup_table[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67};
const uint8_t input_table [5] = {1,2,0,3,4};





#define BUFSIZE 15000

const int ant1_pin = 2;
const int ant2_pin = 3;
const int ant3_pin = 4;
const int ant4_pin =6;

const int numled = 16;
const int neo_pin = 8;

const int TButtonPin = 9;
const int BButtonPin = 15;


Bounce TButton = Bounce();
Bounce BButton = Bounce();

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, neo_pin, WS2812_GRB);

//Lighter versions
#define RED    0x2F0000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x100400
#define PINK   0x120009
#define ORANGE 0x1F0100
#define ORANGER_YELLOW 0x1F0200
#define LIGHT  0x141414
#define DARK   0x000000

int bufferReady;

int intCounter;
int mode, display_pass;
int color;
int pos,correction,corrected;

static bool armed, filter_busy, terse = 0;

static volatile uint16_t head, tail;
static volatile uint16_t buffer[BUFSIZE];

int i, q;

int FUNDIMENTAL_FREQUENCY = 500;   //Doppler tone
int PERIOD = 1000000/(4*FUNDIMENTAL_FREQUENCY);  //in microseconds 
int HARMONIC_FREQUENCY = 2 * FUNDIMENTAL_FREQUENCY;   //First harmonic
double SAMPLING_RATE = 4000;     // in HZ
int numSamples = 4000;          //Number of samples processed

double      k_0,k_1;
double   floatnumSamples, power;
double   omega_0,sine_0,cosine_0,coeff_0,q0_0,q1_0,q2_0,magnitude_0,dop_phase_0,real_0,imag_0;
double   omega_1,sine_1,cosine_1,coeff_1,q0_1,q1_1,q2_1,magnitude_1,dop_phase_1,real_1,imag_1;
double   scalingFactor = numSamples / 2.0;

IntervalTimer sampleTimer;

int16_t adc_read(void)
{            
    uint16_t h, t;
    uint16_t val;
    t = tail;
/*    do {
        h = head;
        t = tail;                   // wait for data in buffer
    } while (h == t); */
    if (++t >= BUFSIZE) t = 0;
    val = buffer[t];                // remove 1 sample from buffer
    tail = t;
    return val;
}


void adc_process_sample(void)
{                
    uint16_t h;
    uint16_t val;
    if (armed == false) { return; }
    val = analogRead(A2);                // grab new reading from ADC  
    h = head + 1;
    //if (h >= BUFSIZE) h = 0; 
    if (h >= numSamples) {  armed = 0; bufferReady = 1;  }
    if (h != tail) {                // if the buffer isn't full
        buffer[h] = val;            // put new data into buffer
        head = h;    
    }              
}

void make_sample(void){
    uint16_t val;
    float correction = numSamples/SAMPLING_RATE;
    for(int h = 0;  h <= numSamples; h++){
      buffer[h]  = 4096 *  (0.5 - (0.5 * cos((correction * 2* M_PI * FUNDIMENTAL_FREQUENCY * h)/(numSamples))) + (0.5 * cos((correction * 2* M_PI * HARMONIC_FREQUENCY * h)/(numSamples))));   // put new data into buffer         
    }
}

void dump_sample(void){
  for(int h = 0;  h <= numSamples; h++){
    Serial.print(buffer[h]); Serial.print(" ");
  }
  Serial.println("");
}

void start_filter(void)
{    
    floatnumSamples = (double) numSamples;
    k_0 = (floatnumSamples * FUNDIMENTAL_FREQUENCY) / SAMPLING_RATE;    //(int) (0.5 + ((floatnumSamples * FUNDIMENTAL_FREQUENCY) / SAMPLING_RATE));                                    ///FIX ME!!!!!!!!!!!!!!!!!!!!!!
    k_1 = (floatnumSamples * HARMONIC_FREQUENCY) / SAMPLING_RATE;   //k_1 = (int) (0.5 + ((floatnumSamples * HARMONIC_FREQUENCY) / SAMPLING_RATE));
    omega_0 = (2.0 * M_PI * k_0) / floatnumSamples;
    omega_1 = (2.0 * M_PI * k_1) / floatnumSamples;

    sine_0 = sin(omega_0);
    sine_1 = sin(omega_1);
    cosine_0 = cos(omega_0);
    cosine_1 = cos(omega_1);
    coeff_0 = 2.0 * cosine_0;
    coeff_1 = 2.0 * cosine_1;
    q0_0=0; q0_1=0;
    q1_0=0; q1_1=0;
    q2_0=0; q2_1=0;

    
    power = 0;
    
    head = 0;   //empty sample buffer  
    tail = 0;
}

void rotate_antenna(void){
                                                                                                                                                                   
   if (++intCounter > 3 ) intCounter = 0;
  {
    switch (intCounter)
    {
      case 0 : 
        digitalWrite(ant1_pin, HIGH); //start timing process with antenna 1 going active
        digitalWrite(ant4_pin, LOW);
        if (bufferReady == 0) {armed = 1;  }
        return;
      case 1 : 
         digitalWrite(ant2_pin, HIGH);  // turn on next antenna before turning off current antenna 
        digitalWrite(ant1_pin, LOW);
        return;
      case 2 : 
        digitalWrite(ant3_pin, HIGH);
        digitalWrite(ant2_pin, LOW);
        return;
      case 3 : 
        digitalWrite(ant4_pin, HIGH);
        digitalWrite(ant3_pin, LOW);
        return;
    }
  }
}  

void colorWipe(int led_color, int led_count) {
  if (led_count >= 0){
    for (int i=0; i <= led_count; i++) leds.setPixel(numled - i -1, led_color);
    for (int i  = led_count + 1; i < numled; i++) leds.setPixel(numled - i -1, 0);}
  else  
    { 
    for (int i=0; i < abs(led_count); i++) leds.setPixel(i, led_color);
    for (int i  = abs(led_count) + 1; i < numled; i++) leds.setPixel(i, 0);  
      } 
  leds.show();  
  }


void  ledsPowerup(){
          colorWipe(BLUE,numled);
          delay(100);
          colorWipe(GREEN,numled);
          delay(100);
          colorWipe(RED,numled);
          delay(100);
          colorWipe(ORANGER_YELLOW,numled);
          delay(100);      
          colorWipe(ORANGER_YELLOW,numled);
          delay(100);
          colorWipe(YELLOW,numled);
          delay(100);
          colorWipe(LIGHT,numled);
          delay(100);   
          colorWipe(DARK,numled);
          delay(100);   
         leds.show(); 
}

void filter() {
      
      double sample;
      const double bias = 2047;
        
     for(i=0; i<numSamples; i++)
      {
          sample = (double) (adc_read() - bias);
          
          if (i % 1 == 0){                         //skip even samples for fundimental
          
          q2_0 = q1_0;
          q1_0 = q0_0;
          q0_0 = sample + coeff_0 * q1_0 - q2_0;
          }
          
          
          q2_1 = q1_1;
          q1_1 = q0_1;
          q0_1 = sample + coeff_1 * q1_1 - q2_1;
          
          power = power + (sample)*(sample);  //power is always positive
          
      }
     
    // calculate the real and imaginary results
    // scaling appropriately
    real_0 = (q0_0 - q1_0 * cosine_0) / scalingFactor;
    real_1 = (q0_1 - q1_1 * cosine_1) / scalingFactor;
    
    imag_0 = (q1_0 * sine_0) / scalingFactor;
    imag_1 = (q1_1 * sine_1) / scalingFactor;



    magnitude_0 = sqrtf(real_0*real_0 + imag_0*imag_0);
    dop_phase_0 = atan2(imag_0,real_0)*180/M_PI + 180;

    magnitude_1 = sqrtf(real_1*real_1 + imag_1*imag_1);

    power = sqrt(power/numSamples);
    
    bufferReady = 0;

}


void ddf_mode(){
    
       if (armed == 0 && bufferReady == 1) {
        
        start_filter();  //initalize
        //make_sample();
        //dump_sample();
      
        filter();        //crunch numbers
        
        if((magnitude_0/power > 0.001) && (magnitude_0/magnitude_1 > 0.01 )) { 
        if (magnitude_0/(power) < 1.1 ){q = 9; color = GREEN;}
        if (magnitude_0/(power) < 1.0  ){q = 8; color = GREEN;}
        if (magnitude_0/(power) < 0.9  ){q = 7; color = YELLOW;}
        if (magnitude_0/(power) < 0.8  ){q = 6; color = YELLOW;}
        if (magnitude_0/(power) < 0.7 ) {q = 5; color = ORANGER_YELLOW;}
        if (magnitude_0/(power) < 0.6 ) {q = 4; color = ORANGER_YELLOW;}
        if (magnitude_0/(power) < 0.5 ) {q = 3; color = ORANGER_YELLOW;}
        if (magnitude_0/(power) < 0.4 ) {q = 2; color = ORANGE;}
        if (magnitude_0/(power) < 0.3 ) {q = 1; color = RED;}
        if (magnitude_0/(power) < 0.2 ) {q = 0; color = RED;}
        }
        else
          {
           q = 0; 
           color = RED;
          };
      
    corrected = dop_phase_0 + correction;   

    corrected = corrected % 360;
    
    if (corrected < 0) {
      corrected = (360 + corrected) % 360;  //goodbye negative degrees
    }

    
    for (i = 0 ; i < numled ; i++ ) leds.setPixel(i,color);  //Set background to display signal quality
    if (corrected > 0 )
             leds.setPixel(abs((-numled  + int(corrected * 16/360)) % numled) ,LIGHT);
          else
            leds.setPixel(abs((int(corrected  * 16/360))) % numled ,LIGHT);
    leds.show();


//   compass.readHeading();

    

 
    //I2C_display(corrected);

    if (terse == 0){  
    Serial.print(" ");
    Serial.print(magnitude_0);
    Serial.print(" ");
    Serial.print(magnitude_1);
    Serial.print(" raw: ");    
    Serial.print(corrected);
    
    Serial.print(" ");    
    Serial.print(q);

    Serial.print(" ");    
    Serial.print(power);

    Serial.print(" ");    
    Serial.print(mode);

    
    Serial.println(" ");

//    Serial.print("Mag: ");
//    Serial.println((compass.heading/10 - 11 )%360);
    //Serial.print(" True: "); 
    //Serial.println((compass.heading/10 + corrected + 11) % 360);  //add declination for the west coast
    }
    
    
    //Serial.print("Free Memory = ");
    //Serial.println(ram.free() / 1024);
     //Serial.println();
     
     
     
     }

     

}


void readHeading(){
    
  //sendCommand(command); // Send specified I2C command to HMC6343
  Wire.beginTransmission(0x19);
  Wire.write(0x50);
  Wire.endTransmission();
  //delay(1); // Delay response time
  unsigned long time = micros();
  while(micros() < time + 1000) ;
  //clearRawData(); // Clear object's rawData[] array before storing new values in the array
  
  // Wait for a 6 byte response via I2C and store them in rawData[] array
  Wire.beginTransmission(0x19);
  Wire.requestFrom(0x19,(uint8_t)6,I2C_STOP, (uint32_t)1000); // Request the 6 bytes of data (MSB comes first)
  //Wire.sendRequest(0x19,(uint8_t)6,I2C_STOP,(uint32_t)100); // Request the 6 bytes of data (MSB comes first)
  uint8_t i = 0;
  while(Wire.available() && i < 6)
  {
    rawData[i] = (uint8_t)Wire.read();
    i++;
  }
  Wire.endTransmission();
  
  // Convert 6 bytes received into 3 integers
  first = rawData[0] << 8; // MSB
  first |= rawData[1];     // LSB
  sec = rawData[2] << 8;
  sec |= rawData[3];
  third = rawData[4] << 8;
  third |= rawData[5];

}


void null_routine(){
  return;
}

void ant_test_mode(){
      
       // if (BButton.update() && BButton.fallingEdge())  {
       if (digitalRead(BButtonPin) == 0)  {
        
        Serial.println("putton pressed ");
        pos = pos + 1;
        pos =  pos % 4;
        Serial.println(pos);}
        
    FrequencyTimer2::setOnOverflow(null_routine);
    colorWipe(DARK,numled);

      
      switch (pos)
    {
      case 0 : 
        leds.setPixel(0, LIGHT);
        leds.show();
        digitalWrite(ant1_pin, HIGH);
        digitalWrite(ant2_pin, LOW);
        digitalWrite(ant3_pin, LOW);
        digitalWrite(ant4_pin, LOW);
        delay(500);
        return;
      case 1 : 
        leds.setPixel(12, LIGHT); 
        leds.show();
        digitalWrite(ant2_pin, HIGH);  
        digitalWrite(ant1_pin, LOW);
        digitalWrite(ant3_pin, LOW);
        digitalWrite(ant4_pin, LOW);
        delay(500);
        return;
      case 2 : 
        leds.setPixel(8, LIGHT);
        leds.show();
        digitalWrite(ant3_pin, HIGH);
        digitalWrite(ant1_pin, LOW);
        digitalWrite(ant2_pin, LOW);
        digitalWrite(ant4_pin, LOW);
        delay(500);
        return;
      case 3 :
        leds.setPixel(4, LIGHT); 
        leds.show();
        digitalWrite(ant4_pin, HIGH);
        digitalWrite(ant1_pin, LOW);
        digitalWrite(ant2_pin, LOW);
        digitalWrite(ant3_pin, LOW);
        delay(500);
        return;
    }  
}

void show(){
  Wire.beginTransmission(addr);
  Wire.write(0x00); // start at address 0x0

  for (int i = 0; i < 8; i++) {
    Wire.write(displayBuffer[i] & 0xFF);    
    Wire.write(displayBuffer[i] >> 8);    
  }
  Wire.endTransmission();  
}

void setBrightness(uint8_t b){
  if(b > 15) return;

  Wire.beginTransmission(addr);
  Wire.write(0xE0 | b); // Dimming command
  Wire.endTransmission();
}

void I2C_display(int angle){
    displayBuffer[0] = 0; //lookup_table[int(angle/1000)];
    angle = angle - int(angle/1000)*1000;
    displayBuffer[1] = lookup_table[int(angle/100)];
    angle = angle - int(angle/100)*100;
    displayBuffer[3] = lookup_table[int(angle/10)];
    angle = angle - int(angle/10)*10;
    displayBuffer[4] = lookup_table[int(angle)];
    show();
}


void setup() {
    Serial.begin(115200);
    Serial.println("Starting");

    Serial1.begin(9600);
    //Serial1.println("Starting");
    
    sampleTimer.begin(adc_process_sample,250);

     pinMode(ant1_pin, OUTPUT);
     pinMode(ant2_pin, OUTPUT);
     pinMode(ant3_pin, OUTPUT);     
     pinMode(ant4_pin, OUTPUT);

    pinMode(5,OUTPUT);   //tied to FrquencyTimer2
    pinMode(7,OUTPUT);   //for debugging
    

    //pinMode(TButtonPin, INPUT_PULLUP);
    //pinMode(BButtonPin, INPUT_PULLUP);

    pinMode(TButtonPin, INPUT_PULLUP);
    TButton.attach(TButtonPin);
    TButton.interval(25);
    pinMode(BButtonPin, INPUT_PULLUP);
    BButton.attach(BButtonPin);
    TButton.interval(25);
   
//  pinMode(11,OUTPUT);   //test signal
    FrequencyTimer2::setPeriod(PERIOD);   //Frequency Timer is master clock, Doppler * 4, in microseconds
    FrequencyTimer2::enable();
    FrequencyTimer2::setOnOverflow(rotate_antenna);

    leds.begin();
    ledsPowerup();
/* 
    // Initialize the HMC6343 and verify its physical presence
    if (!compass.init())
      {
        Serial.println("Sensor Initialization Failed\n\r"); // Report failure, is the sensor wiring correct?
    }
*/

    analogReadResolution(12);  //Changes deault Arduino to maximum ADC resolution

    analogReference(INTERNAL);

}


void loop() {
  //Serial.print(".");
  long newLeft, newRight;
 
    
       const int maxmode = 4;
      TButton.update();
      if (TButton.fell()) {
        Serial.println("putton pressed ");
        mode = mode + 1;
        mode =  mode % maxmode;
      }

      switch(mode){
        case 0: 
                ddf_mode();
          
                 if (BButton.update() && BButton.fell()) {
                 //if (BButton.fallingEdge()) { */
                  Serial.println("putton pressed ");
                  pos = pos + 1;
                  pos =  pos % 4;
                  //Serial.print("%"); Serial.println((compass.heading/10 + corrected +11) % 360);                  
                  //Serial1.print("%"); Serial1.print((compass.heading/10 + corrected + 11) % 360); Serial1.print('\n') ; }}  
                Serial.print("%"); Serial.println((corrected) % 360);                  
                Serial1.print("%"); Serial1.print((corrected) % 360); Serial1.print('\n') ; }
                break;
        
        case 1: ant_test_mode();
                break;
                
        case 2: FrequencyTimer2::setOnOverflow(rotate_antenna);
                ddf_mode();
                if (BButton.update()) {
                  if (BButton.fallingEdge()) {
                  Serial.println("bottom putton pressed ");
                  mode = 0;
                  correction = - dop_phase_0;}}
                  break;
        
        case 3: Serial.print("Doppler Frequency = "); Serial.println(FUNDIMENTAL_FREQUENCY);
                knobLeft.write(0);  
                do {
                  newLeft = knobLeft.read();
                  if (newLeft != positionLeft) {
                    positionLeft = newLeft;
                    FUNDIMENTAL_FREQUENCY  = FUNDIMENTAL_FREQUENCY + newLeft;
                    knobLeft.write(0);  
                    Serial.print("Doppler Frequency = "); Serial.println(FUNDIMENTAL_FREQUENCY);
                    int PERIOD = 1000000/(4*FUNDIMENTAL_FREQUENCY);  //in microseconds 
                    FrequencyTimer2::setPeriod(PERIOD);   //Frequency Timer is master clock, Doppler * 4, in microseconds
                  }
                } while(not(BButton.update()&& BButton.fallingEdge()));
                mode = 0;
                break;
        case 4:
                break;
                
      }



    
    
} 
