
// MCont with MEGA registers and pinout
// Adapted from PGrady EasyController2 - https://github.com/pgrady3/EasyController2

// 3/12/23: MCv5 with original FETs unloaded duration tested

// 2/28/23: MCv5.0 hot-swap battery and max spike tested. Milestone 2 (unloaded, battery) suceeded. old motor halls identified prior, identifyHalls fxn removed. 
// Powering in either order (MC then motor OR motor then MC) both work. plugging in mcu causes motor to immediately act on pot inupts (after start seq).
// Not exceeding 50A unloaded (likely far lower) even on max pot change
// Moderate cogging at <50 PWM

// 2/27/23: MCv5.0 duration tested for 10+ minutes at ~250PWM drawing 24.0V, 0.3A. Power supply
// Moderate cogging at <50 PWM

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <math.h>

// throttle bounding
#define THROTTLE_LOW 30
#define THROTTLE_HIGH 510

// halls, gate driver, throttle, debug pins
#define HALL_1_PIN 2
#define HALL_2_PIN 5
#define HALL_3_PIN 3

#define AH_PIN 44            
#define AL_PIN 8
#define BH_PIN 45
#define BL_PIN 7
#define CH_PIN 46
#define CL_PIN 6

#define THROTTLE_PIN A0      

#define LED_PIN 13

// oversampling for noise filtering during nominal operation. increase if skipping/cogging at low speed
int HALL_OVERSAMPLE = 10;

// mapping hall sensing to motor state
//uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255}; // default hall to motor state mapping, unused
uint8_t hallToMotor[8] = {255, 4, 2, 3, 0, 5, 1, 255}; // correct direction
//uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255}; // reverse direction

// writes pwm to high side FETs, digital value to low side
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl)
{
  // digitalWriteFast(AL_PIN, 0);
  // digitalWriteFast(BL_PIN, 0);
  // digitalWriteFast(CL_PIN, 0);

  analogWrite(AH_PIN, ah);
  analogWrite(BH_PIN, bh);
  analogWrite(CH_PIN, ch);
  digitalWriteFast(AL_PIN, al);
  digitalWriteFast(BL_PIN, bl);
  digitalWriteFast(CL_PIN, cl);
}

// sets corresponding phases to be written given current motor state. if invlaid state, kills all FETs and re-identifies halls.
void writePWM(uint8_t motorState, uint8_t dutyCycle)
{
  if(motorState == 0)                         // LOW A, HIGH B
      writePhases(0, dutyCycle, 0, 1, 0, 0);
  else if(motorState == 1)                    // LOW A, HIGH C
      writePhases(0, 0, dutyCycle, 1, 0, 0);
  else if(motorState == 2)                    // LOW B, HIGH C
      writePhases(0, 0, dutyCycle, 0, 1, 0);
  else if(motorState == 3)                     // LOW B, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 1, 0);
  else if(motorState == 4)                    // LOW C, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 0, 1);
  else if(motorState == 5)                    // LOW C, HIGH B
      writePhases(0, dutyCycle, 0, 0, 0, 1);
  else{                                        // All off
    writePhases(0, 0, 0, 0, 0, 0);
    
    while(true){
      digitalWriteFast(LED_PIN, HIGH);
      delay(1000);
      digitalWriteFast(LED_PIN, LOW);
      delay(1000);
    }
    //identifyHalls();
      
  }
}

// returns currently active halls as 1 in uint8_t, last three digits
uint8_t getHalls()
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++)
  {
    hallCounts[0] += digitalReadFast(HALL_1_PIN);
    hallCounts[1] += digitalReadFast(HALL_2_PIN);
    hallCounts[2] += digitalReadFast(HALL_3_PIN);
  }

  uint8_t hall = 0;
  
  if (hallCounts[0] >= HALL_OVERSAMPLE / 2)     // If votes >= threshold, call that a 1
    hall |= (1<<0);                             // Store a 1 in the 0th bit
  if (hallCounts[1] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<1);                             // Store a 1 in the 1st bit
  if (hallCounts[2] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<2);                             // Store a 1 in the 2nd bit

  return hall & 0x7;                            // Just to make sure we didn't do anything stupid, set the maximum output value to 7
}

void identifyHalls()
{
  HALL_OVERSAMPLE = 10;
  for(uint8_t i = 0; i < 6; i++)
  {
    uint8_t nextState = (i + 1) % 6;          // Calculate what the next state should be. This is for switching into half-states
    
    uint8_t curHalls = getHalls();
    uint8_t attempts = 0;
    
    while ((getHalls() == curHalls))          // new and reliable method, spin until next hall state reached to map
    //for(uint16_t j = 0; j < 300; j++)       // original method: for a while, repeatedly switch between states and assume next reached to map
    {
      delay(1);
      writePWM(i, 50);
      delay(1);
      writePWM(nextState, 50);
      attempts += 1;
    }
    
    Serial.println(attempts);
    writePWM(i,0);
    writePWM(nextState,0);

    // print and stall if next hall not reached in 500 pulses (unreached, untested)
    if (attempts >= 500){
      Serial.print("Init failed with hall activation ");
      Serial.println(getHalls());
      while (true){} // infinite loop
    }
    
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    
    hallToMotor[getHalls()] = (i + 2) % 6;  // Store the hall state - motor state correlation. Notice that +2 indicates 90 degrees ahead, as we're at half states

    for(uint8_t i = 0; i < 8; i++)            // Print out the array
    {
      Serial.print(hallToMotor[i]);
      Serial.print(", ");
    }
    Serial.println(getHalls());
    Serial.println("");
  }
  
  //writePWM(0, 0);                           // Turn phases off
  
  for(uint8_t i = 0; i < 8; i++)            // Print out the array
  {
    Serial.print(hallToMotor[i]);
    Serial.print(", ");
  }
  HALL_OVERSAMPLE = 8;
  Serial.println();
}

uint8_t readThrottle()
{
  int adc = analogRead(THROTTLE_PIN); 
  adc = adc/4; //rescale for /255
  
  // bound/processing if necessary
  adc = (adc - THROTTLE_LOW);
  //adc = adc*2;

  if (adc > 254)
    return 254;

  if (adc < 1)
    return 0;

  return adc;

  // ** static throttle input for testing ** //
  // uint8_t adc = 140;
  // if (random(0, 100) > 80) {
  //   adc = 80;
  // }
  //return adc;
}

void setup() {
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  // Mode 10: phase correct PWM with ICR4 as Top (= F_CPU/2/25000)
  // OC4C as Non-Inverted PWM output
  ICR5   = (F_CPU/32000)/2;
  OCR5C  = ICR5/2;                    // default: about 50:50
  TCCR5A = _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(CS50);
  // Set the PWM pin as output.
  
  Serial.begin(115200);
  Serial.println("mcont testing");
  pinMode(LED_PIN, OUTPUT);
 
  // init sequence for debugging
  digitalWriteFast(LED_PIN, LOW);
  delay(1000);
  digitalWriteFast(LED_PIN, HIGH);
  delay(1000);
  digitalWriteFast(LED_PIN, LOW);
  delay(1000);

  pinMode(AH_PIN, OUTPUT);
  pinMode(AL_PIN, OUTPUT);
  pinMode(BH_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(CH_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);

  // note halls must be pulled up
  pinMode(HALL_1_PIN, INPUT_PULLUP);
  pinMode(HALL_2_PIN, INPUT_PULLUP);
  pinMode(HALL_3_PIN, INPUT_PULLUP);
              
  pinMode(THROTTLE_PIN, INPUT);
  
  //identifyHalls(); // uncomment for hall identification on startup. suggest doing auto-identify unloaded, import vals into mapping array for loaded
}

// vals used in main loop
uint8_t newThrottle;
uint8_t throttle = 0;
uint8_t motorState;
uint8_t hall;

void loop() {

  //Serial.println(hallToMotor[getHalls()]);
  //Serial.println(readThrottle());

  // // ensure new throttle doesn't suddenly spike PWM, (max spike as of now, 255/2 -> 177
  throttle = readThrottle();
  // //newThrottle = readThrottle();  
  // // if (newThrottle - throttle > 50){
  // //   throttle = throttle + 50;
  // // }
  // // else{
  // //   throttle = newThrottle;
  // // }
  
  // // readThrottle() is slow. So do the more important things 200 times more often
  for(uint8_t i = 0; i < 200; i++)
  { 
    // read current hall state, map to motor state and drive accordingly
    hall = getHalls();
    motorState = hallToMotor[hall]; 
    //writePWM(motorState, throttle);

    // testing with one phase
    writePWM(motorState, throttle);
  }
  
}
