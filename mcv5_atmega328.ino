// Adapted from PGrady EasyController2 - https://github.com/pgrady3/EasyController2
// Adapted for atmega328: Uno board.

// 2/27/23: MCv5.0 duration tested for 10+ minutes at ~250PWM drawing 24.0V, 0.3A.
// Moderate cogging at <50 PWM

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <math.h>

// throttle bounding
#define THROTTLE_LOW 50
#define THROTTLE_HIGH 510

// halls, gate driver, throttle, debug pins
#define HALL_1_PIN 3
#define HALL_2_PIN 5
#define HALL_3_PIN 4

#define AH_PIN 9            
#define AL_PIN 8
#define BH_PIN 10
#define BL_PIN 7
#define CH_PIN 11
#define CL_PIN 6

#define THROTTLE_PIN A0      

#define LED_PIN 13

// oversampling for noise filtering during nominal operation. increase if skipping/cogging at low speed
int HALL_OVERSAMPLE = 10;

// mapping hall sensing to motor state
uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255};

// writes pwm to high side FETs, digital value to low side
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl)
{
  analogWrite(AH_PIN, ah);
  analogWrite(BH_PIN, bh);
  analogWrite(CH_PIN, ch);
  digitalWrite(AL_PIN, al);
  digitalWrite(BL_PIN, bl);
  digitalWrite(CL_PIN, cl);
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
  else if(motorState == 3)                    // LOW B, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 1, 0);
  else if(motorState == 4)                    // LOW C, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 0, 1);
  else if(motorState == 5)                    // LOW C, HIGH B
      writePhases(0, dutyCycle, 0, 0, 0, 1);
  else{                                        // All off
      writePhases(0, 0, 0, 0, 0, 0);
      identifyHalls();
  }
}

// returns currently active halls as 1 in uint8_t, last three digits
uint8_t getHalls()
{
  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // read halls the number of times specified in oversample
  {
    hallCounts[0] += digitalRead(HALL_1_PIN);
    hallCounts[1] += digitalRead(HALL_2_PIN);
    hallCounts[2] += digitalRead(HALL_3_PIN);
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

/* Magic function to do hall auto-identification. Moves the motor to all 6 states, then reads the hall values from each one
 * 
 * Note, that in order to get a clean hall reading, we actually need to commutate to half-states. So instead of going to state 3, for example
 * we commutate to state 3.5, by rapidly switching between states 3 and 4. After waiting for a while (half a second), we read the hall value.
 * Finally, print it
 */
void identifyHalls()
{
  HALL_OVERSAMPLE = 32;
  for(uint8_t i = 0; i < 6; i++)
  {
    uint8_t nextState = (i + 1) % 6;        // Calculate what the next state should be. This is for switching into half-states
    for(uint16_t j = 0; j < 300; j++)       // For a while, repeatedly switch between states
    {
      delay(1);
      writePWM(i, 30);
      delay(1);
      writePWM(nextState, 30);
    }
    writePWM(0,0);
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
  
  writePWM(0, 0);                           // Turn phases off
  
  for(uint8_t i = 0; i < 8; i++)            // Print out the array
  {
    Serial.print(hallToMotor[i]);
    Serial.print(", ");
  }
  HALL_OVERSAMPLE = 8;
  Serial.println();
}

/* Read the throttle value from the ADC. Because our ADC can read from 0v-3.3v, but the throttle doesn't output over this whole range,
 * scale the throttle reading to take up the full range of 0-255
 */
uint8_t readThrottle()
{
  unsigned int adc = analogRead(THROTTLE_PIN); 
  adc = adc/4; //rescale for /255
  
  // bound/processing if necessary
  //adc = (adc - THROTTLE_LOW);
  adc = adc*2;

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
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  
  Serial.begin(115200);
  Serial.println("mcont testing");
  pinMode(LED_PIN, OUTPUT);
 
  // init sequence for debugging
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
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
  
  identifyHalls();
}

// vals used in main loop
uint8_t newThrottle;
uint8_t throttle = 0;
uint8_t motorState;
uint8_t hall;

void loop() {
  // ensure new throttle doesn't suddenly spike PWM, (max spike as of now, 255/2 -> 177
  newThrottle = readThrottle();  
  if (newThrottle - throttle > 100){
    throttle = throttle + round(0.5*(newThrottle-throttle));
  }
  else{
    throttle = newThrottle;
  }
  
  // readThrottle() is slow. So do the more important things 200 times more often
  for(uint8_t i = 0; i < 200; i++)
  { 
    // read current hall state, map to motor state and drive accordingly
    hall = getHalls();
    motorState = hallToMotor[hall]; 
    writePWM(motorState, throttle);
  }
  
}
