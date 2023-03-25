// MContV6_2 pinout
// Adapted from PGrady EasyController2 - https://github.com/pgrady3/EasyController2

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <math.h>

// throttle bounding
#define THROTTLE_LOW 30
#define THROTTLE_HIGH 510

// halls, gate driver, throttle, debug pins
#define HALL_A_PIN 2
#define HALL_B_PIN 3
#define HALL_C_PIN 5

// spare PWM pins
#define PWM1 6
#define PWM2 7
#define PWM3 8

// spare analog pins
#define ANLG1 A3
#define ANLG2 A2
#define ANLG3 A1

// driver control pins
#define AH_PIN 10   
#define AL_PIN 12
#define BH_PIN 45 
#define BL_PIN 46
#define CH_PIN 44
#define CL_PIN 43 

// overcurrent (active high) pin
#define OVERC 21 // formerly 42 (no interrupt)

#define THROTTLE_PIN A4      

#define LED_PIN 22

// oversampling for noise filtering during nominal operation. increase if skipping/cogging at low speed
int HALL_OVERSAMPLE = 10;


uint8_t motorState;

// mapping hall sensing to motor state
//uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255}; // default hall to motor state mapping, unused
uint8_t hallToMotor[8] = {255, 4, 2, 3, 0, 5, 1, 255}; // correct direction

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
  
  // spin shaft clockwise viewing from shaft to motor
  if(motorState == 0)                         // LOW A, HIGH B
      writePhases(0, dutyCycle, 0, 1, 0, 0);
  else if(motorState == 1)                    // LOW A, HIGH C
      writePhases(0, 0, dutyCycle, 1, 0, 0);
  else if(motorState == 2)                    // LOW B, HIGH C
      writePhases(0, 0, dutyCycle, 0, 1, 0);
  else if(motorState == 3)                    // LOW B, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 1, 0);
  else if(motorState == 4)                    // LOW C, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 0, 1);
  else if(motorState == 5)                    // LOW C, HIGH B
      writePhases(0, dutyCycle, 0, 0, 0, 1);
  else{                                      
    writePhases(0, 0, 0, 0, 0, 0);
    
    // hang program and express failure condition
    while(true){
      digitalWriteFast(LED_PIN, HIGH);
      delay(1000);
      digitalWriteFast(LED_PIN, LOW);
      delay(1000);
    }    
  }
}

// returns currently active halls as 1 in uint8_t, last three digits
uint8_t getHalls()
{

  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // read halls the number of times specified in oversample
  {
    hallCounts[0] += digitalReadFast(HALL_C_PIN);
    hallCounts[1] += digitalReadFast(HALL_A_PIN);
    hallCounts[2] += digitalReadFast(HALL_B_PIN);
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

/* Auto-identifcation. Moves the motor to all 6 states, then reads the hall values from each one
 * 
 * Note, that in order to get a clean hall reading, we actually need to commutate to half-states. So instead of going to state 3, for example
 * we commutate to state 3.5, by rapidly switching between states 3 and 4. After waiting for a while (half a second), we read the hall value.
 * Finally, print it
 */
void identifyHalls()
{
  HALL_OVERSAMPLE = 10;
  for(uint8_t i = 0; i < 6; i++)
  {
    uint8_t nextState = (i + 1) % 6;        // Calculate what the next state should be. This is for switching into half-states
    
    uint8_t curHalls = getHalls();
    uint8_t attempts = 0;
    
    while ((getHalls() == curHalls)) // new and reliable method, spin until next hall state reached to map
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
    
    digitalWriteFast(LED_PIN, HIGH);
    delay(500);
    digitalWriteFast(LED_PIN, LOW);
    
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
  // read throttle and rescale for /255
  int adc = analogRead(THROTTLE_PIN); 
  adc = adc/4;
  
  // eliminate zero offset if present / eliminate low PWM range
  adc = (adc - THROTTLE_LOW);

  // bound throttle to prevent overflow
  if (adc > 254)
    return 254;

  if (adc < 1)
    return 0;

  return adc;
}

void checkOverC(){
  // uint8_t start = micros();
  digitalWriteFast(LED_PIN, HIGH);
  digitalWriteFast(AH_PIN, 0);
  digitalWriteFast(BH_PIN, 0);
  digitalWriteFast(CH_PIN, 0);

  // if (digitalReadFast(OVERC)){



  //   // if (motorState == 0 | motorState == 5){
  //   //   digitalWriteFast(BH_PIN, 0);
  //   //   //while (micros() - start < 1){}
  //   //   //digitalWriteFast(BL_PIN, 1);
  //   // }
  //   // else if (motorState == 1 | motorState == 2){
  //   //   digitalWriteFast(CH_PIN, 0);
  //   //   //while (micros() - start < 1){}
  //   //   //digitalWriteFast(CL_PIN, 1);
  //   // }
  //   // else if (motorState == 3 | motorState == 4){
  //   //   digitalWriteFast(AH_PIN, 0);
  //   //   //while (micros() - start < 1){}
  //   //   //digitalWriteFast(AL_PIN, 1);
  //   // }
  // } 
  // else{
  //   digitalWriteFast(LED_PIN, LOW);
  // }
}

void setup() {
  // Set the PWM pin as output.
  TCCR4B = TCCR4B & B11111000 | B00000001; // set digital 6,7,8 to pwm 31372.55 Hz
  TCCR5B = TCCR5B & B11111000 | B00000001; // set digital 44, 45, 46 to pwm 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001; // set digital 9, 10 to pwm 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001;  // set digital 11, 12 to pwm 31372.55 Hz
  
  Serial.begin(115200);
  Serial.println("mcont testing");
  pinModeFast(LED_PIN, OUTPUT);
 
  // init sequence for debugging
  digitalWriteFast(LED_PIN, LOW);
  delay(1000);
  digitalWriteFast(LED_PIN, HIGH);
  delay(1000);
  digitalWriteFast(LED_PIN, LOW);
  delay(1000);

  pinModeFast(PWM1, OUTPUT);
  pinModeFast(PWM2, OUTPUT);
  pinModeFast(PWM3, OUTPUT);

  pinModeFast(ANLG1, OUTPUT);
  pinModeFast(ANLG2, OUTPUT);
  pinModeFast(ANLG3, OUTPUT);

  pinModeFast(AH_PIN, OUTPUT);
  pinModeFast(AL_PIN, OUTPUT);
  pinModeFast(BH_PIN, OUTPUT);
  pinModeFast(BL_PIN, OUTPUT);
  pinModeFast(CH_PIN, OUTPUT);
  pinModeFast(CL_PIN, OUTPUT);

  // note halls must be pulled up
  pinModeFast(HALL_C_PIN, INPUT_PULLUP);
  pinModeFast(HALL_A_PIN, INPUT_PULLUP);
  pinModeFast(HALL_B_PIN, INPUT_PULLUP);
  
  // throttle pin pulled down on board
  pinModeFast(THROTTLE_PIN, INPUT);

  // Formerly overcurrent pin is input, now interrupt
  //pinModeFast(OVERC, INPUT);
  pinMode(OVERC, INPUT);
  attachInterrupt(digitalPinToInterrupt(OVERC), checkOverC, RISING); // commmenting this disables interrupts

  // Uncomment for hall identification on startup. suggest doing auto-identify unloaded, import vals into mapping array for loaded.
  //identifyHalls(); 

  // Uncomment for ramp up sequence to constant speed if loop PWM is constant
  uint8_t hall;
  for(int i = 0; i < 10000; i++)
  { 
    // read current hall state, map to motor state and drive accordingly
    hall = getHalls();
    motorState = hallToMotor[hall]; 
    writePWM(motorState, i/40);
  }

}

// vals used in main loop
uint8_t newThrottle;
uint8_t throttle = 0;
uint8_t hall;
int iterator = 0;

void loop() {

  // uncomment for verification of hitting main loop
  // writePWM(255, 0);
  // while (true){}

  // Uncomment for debugging hall connections
  //Serial.println(hallToMotor[getHalls()]);
  //Serial.println(getHalls());
  //Serial.println(readThrottle());
  
  digitalWriteFast(LED_PIN, LOW);
  
  // prevent spiking of current due to sudden changes in throttle
  // newThrottle = readThrottle();
  // if (newThrottle > throttle){
  //   if (throttle < newThrottle - 10){
  //     throttle = throttle + 10;
  //   }
  //   else{
  //     throttle = newThrottle;
  //   }
  // }
  // else{
  //   if (newThrottle < throttle - 10){
  //     throttle = throttle - 10;
  //   }
  //   else{
  //     throttle = newThrottle;
  //   }
  // }

  
  // for every 200 PWM writes, update throttle
  for(uint8_t i = 0; i < 200; i++)
  { 

    // read current hall state, map to motor state and drive accordingly
    hall = getHalls();
    motorState = hallToMotor[hall]; 

    // uncomment to verify overcurrent pin live
    //checkOverC();

    // uncomment for set speed
    writePWM(motorState, 250);

    // uncomment for throttle speed
    // writePWM(motorState, throttle);
  }
  
}