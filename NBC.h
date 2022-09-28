/*
 *  Narfduino Libraries - Narfduino Brushless Compleat
 *  
 *  Use this to easily control the functions of a Narfduino Brushless Compleat
 *  Specifically for Flyshot enabled 
 *  
 *  (c) 2021 - Ireland Software
 *  License: 
 *    - You are free to use this software for non-commercial use.
 *    - If you purchase a Narfduino Brushless Compleat from Airzone's Blasters (or other authorised supplier), 
 *      then you can use this software commercially for software loaded on that board.
 *    - Commercial use on other boards requires a paid license
 *    - Modifications to the software follow this license
 *    
 *  Warranty:
 *    - No warranty, expressed or implied. Software is provided as-is. Use at own risk.
 *  
 */
 
 #ifndef __LIBNBC_H
 
 #define __LIBNBC_H
 
 #include "Arduino.h"
 
 
 // Modify these as required... 

#ifndef MOTOR_ON_SIGNAL
  #define MOTOR_ON_SIGNAL 2000
#endif
#ifndef MOTOR_OFF_SIGNAL
  #define MOTOR_OFF_SIGNAL 1000
#endif
#ifndef MOTOR_P
  #define MOTOR_P 14
#endif
#ifndef FLYSHOT_DIGITAL_1
  #define FLYSHOT_DIGITAL_1 800 // 400us
#endif
#ifndef FLYSHOT_DIGITAL_0
  #define FLYSHOT_DIGITAL_0 200 // 100us
#endif
#ifndef FLYSHOT_RETRANSMITS
  #define FLYSHOT_RETRANSMITS 5
#endif
#ifndef PIN_ESC1
  #define PIN_ESC1 9
#endif
#ifndef PIN_ESC2
  #define PIN_ESC2 10
#endif
#ifndef PIN_TACH1
  #define PIN_TACH1 8
#endif 
#ifndef PIN_TACH2
  #define PIN_TACH2 7
#endif


// Modify these with extreme caution
/*
 * Set Pin State
 * 
 * Valid for:
 * PC1 P-BRAKE
 * PD5 P-RUN
 * PB1/OC1A F-RUN
 */
#define PIN_BRIDGE_RUN 5
#define PIN_BRIDGE_BRAKE 15
#define SET_P_HIGH_FULL_ON (PORTC |= 0b00000010)
#define SET_P_HIGH_FULL_OFF (PORTC &= 0b11111101)
#define SET_P_LOW_FULL_ON (PORTD |= 0b00100000)
#define SET_P_LOW_FULL_OFF (PORTD &= 0b11011111)
#define GET_BRIDGE_ON_STATE ((PORTD & 0b00100000))
#define GET_BRIDGE_BRAKE_STATE ((PORTC & 0b00000010))
#define TRANSITION_IDLE 0
#define TRANSITION_RUN 1
#define TRANSITION_BRAKE 2
volatile byte NBC_DeadtimeTransition = TRANSITION_IDLE;
volatile bool NBC_PusherLocked = false; // Use this to ensure that we don't try and do 2 things at once.
#define DEADTIME_RUN_TO_BRAKE 3
#define DEADTIME_BRAKE_TO_RUN 5
byte NBC_DeadtimeValue;

// 
#define PIN_BATTERY A0

// Try not to modify this stuff


// Flyshot
#define MOTOR_P_DIV2 (MOTOR_P / 2)
volatile bool NBC_FlyshotTransmitRequest = false; // Use this to indicate that we should transmit a flyshot packet
volatile bool NBC_TransmittingFlyshot = false; // Use this to indicate that the transmission is in progress
volatile unsigned long NBC_OCR1A_Temp;
volatile unsigned long NBC_OCR1B_Temp; 
volatile byte NBC_CurrentFlyshotBit = 0;
volatile unsigned int NBC_CurrentFlyshotPacket = 0;
volatile byte NBC_TargetESC = 0;

#define FLYSHOT_ESC_BOTH 0b00000011
#define FLYSHOT_ESC_A 0b00000001
#define FLYSHOT_ESC_B 0b00000010
#define FLYSHOT_ESC_A_BIT 0
#define FLYSHOT_ESC_B_BIT 1



// Tach controls
volatile unsigned long NBC_Tach1PulseLength = 0;
volatile unsigned long NBC_Tach2PulseLength = 0;
volatile bool NBC_ReadTach1 = false; // Set this to true to get a read from the tach.
volatile bool NBC_ReadTach2 = false;
unsigned long NBC_Tach1RunningSpeed = 0;
unsigned long NBC_Tach2RunningSpeed = 0;
unsigned long NBC_CalibratedTach1Speed = 0;  // This is the calculated Tach 1 speed to match to consider running (+/- variance)
unsigned long NBC_CalibratedTach2Speed = 0; // This is the calculated Tach 2 speed to match to consider running (+/- variance)
#define TACH_VARIANCE 20  // Variance to the calculated 
#define TACH_TIMEOUT 500  // If there are no reads within this time, reset RunningSpeed
#define TACH_CAL_ATTEMPS 100  // Total calibration attempts
#define TACH_CAL_SPINUP 400 // Initial Calibration Spin-up delay
#define TACH_CAL_WAIT 10 // Calibration wait delay
#define TACH_GOOD_READS 10
bool NBC_Flywheel1Good = false; //front
bool NBC_Flywheel2Good = false; //front
#define FLYWHEELS_GOOD (NBC_Flywheel1Good && NBC_Flywheel2Good)
volatile byte NBC_LastPINB;
volatile byte NBC_LastPIND;


// Deadtime timer
ISR( TIMER2_OVF_vect )
{
  TIMSK2 = (0 << TOIE2); // Turn off Overflow Interrupt
  if( NBC_DeadtimeTransition == TRANSITION_RUN )
  {
    // First check if there the brake is running still
    if( GET_BRIDGE_BRAKE_STATE )
    {
      // If it is, turn off the brake. Let idle to stop
      SET_P_HIGH_FULL_OFF;
    }
    else
    {
      // Run the motor
      SET_P_LOW_FULL_ON;
    }
  }
  else if( NBC_DeadtimeTransition == TRANSITION_BRAKE )
  {
    // First check if the motor is running still
    if( GET_BRIDGE_ON_STATE )
    {
      // Turn the motor off, let it idle
      SET_P_LOW_FULL_OFF;
    }
    else
    {
      // Hit the brakes;
      SET_P_HIGH_FULL_ON;
    }
  }
  // Else - IDLE
  NBC_PusherLocked = false;
}


// Updates the PWM Timers
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2); // Adjust for the prescalar
  OCR1A = NewSpeed;
  OCR1B = NewSpeed;
}

void FlyshotStartMotors()
{
  UpdatePWM( MOTOR_ON_SIGNAL );
}

void FlyshotStopMotors()
{
  UpdatePWM( MOTOR_OFF_SIGNAL );
}

inline void ReadTach1() __attribute__((always_inline));
void ReadTach1()
{
  // Tach1 is PB3, capture rising edge

  static unsigned long LastTach1Pulse = 0; 
  static unsigned long Tach1Pulse = 0;   
  if( ((PINB & 0b00000001) == 0b00000001) && ((NBC_LastPINB & 0b00000001) != 0b00000001) )
  {
    //We want to read the tach
    Tach1Pulse = micros();
    if( ReadTach1 )
    {
      NBC_Tach1PulseLength = Tach1Pulse - LastTach1Pulse;
      NBC_ReadTach1 = false;
    }
    LastTach1Pulse = Tach1Pulse;
  }
  NBC_LastPINB = PINB;
}

inline void ReadTach2() __attribute__((always_inline));
void ReadTach2()
{
  // Tach2 is PD7, capture rising edge

  static unsigned long LastTach2Pulse = 0;
  static unsigned long Tach2Pulse = 0;  
  if( ((PIND & 0b10000000) == 0b10000000) && ((NBC_LastPIND & 0b10000000) != 0b10000000) )
  {
    //We want to read the tach
    Tach2Pulse = micros();
    if( ReadTach2 )
    {      
      NBC_Tach2PulseLength = Tach2Pulse - LastTach2Pulse;
      NBC_ReadTach2 = false;
    }
    LastTach2Pulse = Tach2Pulse;
  }
  NBC_LastPIND = PIND;
}

// Keep tabs of the flywheel tach speed. Reset to 0 after a period of time.
void NBCProcessFlywheelSpeed()
{
  static unsigned long NBC_LastTach1Read = 0;
  static bool NBC_LastReadTach1 = true;
  if( !NBC_ReadTach1 )
  {
    // Check for new measurement
    if( NBC_LastReadTach1 )
    {
      // Build it into the average
      if( NBC_Tach1RunningSpeed == 0 )
        NBC_Tach1RunningSpeed = NBC_Tach1PulseLength;
      else
        NBC_Tach1RunningSpeed = (NBC_Tach1RunningSpeed + NBC_Tach1PulseLength) / 2;
    }
    NBC_ReadTach1 = true; // Request a new read
    NBC_LastTach1Read = millis();
  }
  if( millis() - NBC_LastTach1Read > TACH_TIMEOUT )
    NBC_Tach1RunningSpeed = 0;
  NBC_LastReadTach1 = NBC_ReadTach1;

  static unsigned long NBC_LastTach2Read = 0;
  static bool NBC_LastReadTach2 = true;
  if( !NBC_ReadTach2 )
  {
    // Check for new measurement
    if( NBC_LastReadTach2 )
    {
      // Build it into the average
      if( NBC_Tach2RunningSpeed == 0 )
        NBC_Tach2RunningSpeed = NBC_Tach2PulseLength;
      else
        NBC_Tach2RunningSpeed = (NBC_Tach2RunningSpeed + NBC_Tach2PulseLength) / 2;
    }
    NBC_ReadTach2 = true; // Request a new read
    NBC_LastTach2Read = millis();
  }
  if( millis() - NBC_LastTach2Read > TACH_TIMEOUT )
    NBC_Tach2RunningSpeed = 0;  
  NBC_LastReadTach2 = NBC_ReadTach2;  
  
  // Now compare to the calibrated values (provided there are some)
  static byte NBC_Tach1GoodReads = 0;
  if( NBC_Tach1RunningSpeed > 0 && NBC_CalibratedTach1Speed > TACH_VARIANCE )
  {
    if( (NBC_Tach1RunningSpeed > (NBC_CalibratedTach1Speed - TACH_VARIANCE)) && (NBC_Tach1RunningSpeed < (NBC_CalibratedTach1Speed + TACH_VARIANCE)) )
    {
      if( NBC_Tach1GoodReads <= TACH_GOOD_READS ) 
      {
        NBC_Tach1GoodReads ++;
      }
      else
      {
        NBC_Flywheel1Good = true;
      }
    }
    else
    {
      NBC_Flywheel1Good = false;
      NBC_Tach1GoodReads = 0;
    }
  }
  else
  {
    NBC_Flywheel1Good = false;
    NBC_Tach1GoodReads = 0;
  }

  static byte NBC_Tach2GoodReads = 0;
  if( NBC_Tach2RunningSpeed > 0 && NBC_CalibratedTach2Speed > TACH_VARIANCE )
  {
    if( (NBC_Tach2RunningSpeed > (NBC_CalibratedTach2Speed - TACH_VARIANCE)) && (NBC_Tach2RunningSpeed < (NBC_CalibratedTach2Speed + TACH_VARIANCE)) )
    {
      if( NBC_Tach2GoodReads <= TACH_GOOD_READS ) 
      {
        NBC_Tach2GoodReads ++;
      }
      else
      {
        NBC_Flywheel2Good = true;
      }
    }
    else
    {
      NBC_Flywheel2Good = false;
      NBC_Tach2GoodReads = 0;
    }
  }
  else
  {
    NBC_Flywheel2Good = false;
    NBC_Tach2GoodReads = 0;
  }  

}


// Run the flywheels for a brief moment to calibrate the tach
// ** BLOCKING **
bool CalibrateFlywheels()
{  
  NBC_CalibratedTach1Speed = 0;
  NBC_CalibratedTach2Speed = 0;
  unsigned long CalStartTime = millis();

  // Spin-up motor, keeping an eye on the Tach
  FlyshotStartMotors();
  while( (millis() - CalStartTime) < TACH_CAL_SPINUP )
  {
    delay(1);
    NBCProcessFlywheelSpeed();
  }

  // Set the calibrated period
  NBC_CalibratedTach1Speed = NBC_Tach1RunningSpeed;
  NBC_CalibratedTach2Speed = NBC_Tach2RunningSpeed;

  for( int CalAttempt = 0; CalAttempt < TACH_CAL_ATTEMPS; CalAttempt++ )
  {
    // Wait for stable readings
    CalStartTime = millis();
    while( (millis() - CalStartTime) < TACH_CAL_WAIT )
    {
      delay(1);
      NBCProcessFlywheelSpeed();
    }
    
    // Flywheels are good, you can exit now.
    if( NBC_Flywheel1Good && NBC_Flywheel2Good )
    {
      FlyshotStopMotors();
      
      return true;
    }

    // Reset the calibration
    NBC_CalibratedTach1Speed = NBC_Tach1RunningSpeed;
    NBC_CalibratedTach2Speed = NBC_Tach2RunningSpeed; 
  }

  // Flywheels are no good.
  FlyshotStopMotors();
  
  return false;  
}


// Tach Functions
#ifndef NBC_OVERRIDE_PCINT
// Use the PCINT vectors to capture the pulse duration for RPM monitoring
// PCINT - PB
ISR( PCINT0_vect ) // PB
{
  ReadTach1();
}
// PCINT - PD
ISR( PCINT2_vect ) // PD
{
  ReadTach2();
}
#endif

// Flyshot Communications
ISR( TIMER1_COMPA_vect )
{
  if( NBC_FlyshotTransmitRequest )
  {
    NBC_TransmittingFlyshot = true;
    if( NBC_CurrentFlyshotBit > 0 )
    {
      NBC_CurrentFlyshotBit --;
      if( NBC_CurrentFlyshotPacket & (0x0001 << NBC_CurrentFlyshotBit) )
      {
        // High
        OCR1A = FLYSHOT_DIGITAL_1;
        if( bitRead(NBC_TargetESC, FLYSHOT_ESC_B_BIT) ) OCR1B = FLYSHOT_DIGITAL_1;
      }
      else
      {
        // Low
        OCR1A = FLYSHOT_DIGITAL_0;
        if( bitRead(NBC_TargetESC, FLYSHOT_ESC_B_BIT) ) OCR1B = FLYSHOT_DIGITAL_0;        
      }
    }
    else
    {
      // Restore OCR1x
      OCR1A = NBC_OCR1A_Temp;
      if( bitRead(NBC_TargetESC, FLYSHOT_ESC_B_BIT) ) OCR1B = NBC_OCR1B_Temp;
      // We are done
      NBC_TransmittingFlyshot = false;
      NBC_FlyshotTransmitRequest = false;
    }
  }
}

ISR( TIMER1_COMPB_vect )
{
  if( NBC_FlyshotTransmitRequest )
  {
    NBC_TransmittingFlyshot = true;
    if( NBC_CurrentFlyshotBit > 0 )
    {
      NBC_CurrentFlyshotBit --;
      if( NBC_CurrentFlyshotPacket & (0x0001 << NBC_CurrentFlyshotBit) )
      {
        // High
        OCR1B = FLYSHOT_DIGITAL_1;
      }
      else
      {
        // Low
        OCR1B = FLYSHOT_DIGITAL_0;        
      }
    }
    else
    {
      // Restore OCR1x
      OCR1B = NBC_OCR1B_Temp;
      // We are done
      NBC_TransmittingFlyshot = false;
      NBC_FlyshotTransmitRequest = false;
    }
  }
}

// *** Blocking ***
// Sets new flyshot para
void FlyshotSetParameters( unsigned int FlyshotPacket, byte TargetESC )
{ 
  NBC_OCR1A_Temp = OCR1A;
  NBC_OCR1B_Temp = OCR1B;

  // Enable Governor Interrupt
  cli();
  NBC_TargetESC = TargetESC;
  if( (TargetESC && FLYSHOT_ESC_A) == FLYSHOT_ESC_A ) TIMSK1 = ( 1 << OCIE1A ); // If A or Both then use this one 
  if( TargetESC == FLYSHOT_ESC_B ) TIMSK1 = ( 1 << OCIE1B ); // If only B then use this one
  //PCICR &= ~(0 << PCIE0); // Dectivates control register for PCINT vector 0
  sei();
  delay( 10 ); // Settle for a bit

  // Need to transmit Flyshot a number of times to ensure stability.
  for( int Pass = 0; Pass < FLYSHOT_RETRANSMITS; Pass++ )
  {
    NBC_CurrentFlyshotBit = 16;
    NBC_CurrentFlyshotPacket = FlyshotPacket;
    NBC_FlyshotTransmitRequest = true;
  
    while( NBC_FlyshotTransmitRequest || NBC_TransmittingFlyshot )
    {
      delayMicroseconds( 1000 );
    }
  }
 
  delay( 10 );
  // Disable Governor Interrupt
  cli();
  NBC_TargetESC = 0;
  if( (TargetESC && FLYSHOT_ESC_A) == FLYSHOT_ESC_A ) TIMSK1 = (0 << OCIE1A );
  if( TargetESC == FLYSHOT_ESC_B ) TIMSK1 = ( 1 << OCIE1B );
  //PCICR |= (1 << PCIE0); // Activates control register for PCINT vector 0
  sei();

}

void FlyshotSetNewMotorSpeed( unsigned long MotorRPM )
{
  unsigned long SetPoint = (unsigned long)320000000 / (MotorRPM * MOTOR_P_DIV2);
  unsigned int FlyshotGovernorFrame = SetPoint | 0x8000;

  FlyshotSetParameters( FlyshotGovernorFrame, FLYSHOT_ESC_BOTH );
}

void FlyshotSetMotorDirectionForward( byte TargetESC )
{
  unsigned int FlyshotParamFrame = 0b0111101010000000;
  
  FlyshotSetParameters( FlyshotParamFrame, TargetESC );
}

void FlyshotSetMotorDirectionReverse( byte TargetESC )
{
  unsigned int FlyshotParamFrame = 0b0111101011000000;
  
  FlyshotSetParameters( FlyshotParamFrame, TargetESC );
}

void FlyshotBeep1( byte TargetESC )
{
  unsigned int FlyshotParamFrame = 0b0111101000100000;
  
  FlyshotSetParameters( FlyshotParamFrame, TargetESC );
}

void FlyshotBeep2( byte TargetESC )
{
  unsigned int FlyshotParamFrame = 0b0111101000110000;
  
  FlyshotSetParameters( FlyshotParamFrame, TargetESC );
}

void NBCInit()
{
  pinMode( PIN_ESC1, OUTPUT );
  pinMode( PIN_ESC2, OUTPUT );
  PORTD &= 0b11001111;
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 5000; // 400hz
  UpdatePWM( MOTOR_OFF_SIGNAL );  

  NBC_LastPIND = PIND;
  NBC_LastPINB = PINB;
  pinMode(PIN_TACH1, INPUT_PULLUP); 
  PCMSK0 |= (1 << PCINT0);
  pinMode(PIN_TACH2, INPUT_PULLUP); 
  PCMSK2 |= (1 << PCINT23);    
  PCICR |= (1 << PCIE0); // Activates control register for PCINT vector 0
  PCICR |= (1 << PCIE2); // Activates control register for PCINT vector 2    

  pinMode( PIN_BRIDGE_RUN, OUTPUT );
  digitalWrite( PIN_BRIDGE_RUN, LOW );
  
  pinMode( PIN_BRIDGE_BRAKE, OUTPUT );
  digitalWrite( PIN_BRIDGE_BRAKE, LOW );

  // Setup Deadtime Timer
  /*
    Prescalar /1024
    Each 1 = 64ns
    Count to 10 = 640ns
    Off time + Falltime for WSD30L120DN56 + 2N7002 = 270ns + 15ns = 285ns
    On Time + Rise Time for WSD30L120DN56 + 2N7002 = 50ns + 10ns
    Brake to Run deadtime = 285ns; OCR2A = 5 (320ns)
    
    Off time + fall time for PSMN1R0-30YLC = 170ns 
    On time + rise time for PSMN1R0-30YLC = 120ns
    Run to Brake deadtime = 170ns; OCR2A =  3 (192ns)
    
    FastPWM   
*/
  noInterrupts(); // Timer2 doesn't like reconfiguration if interrupts are enabled.. 
  TCCR2A = 0;
  TCCR2A = (1 << WGM20) | (1 << WGM21);
  TCCR2B = 0;
  TCCR2B = (1 << WGM22) | (1 << CS20) | (1 << CS21) | (1 << CS22);
  OCR2A = 100;    
  interrupts();  

  pinMode( PIN_BATTERY, INPUT );
}


void NBCSolenoidOn()
{
  SET_P_HIGH_FULL_OFF;
  SET_P_LOW_FULL_ON;
}

void NBCSolenoidOff()
{
  SET_P_HIGH_FULL_OFF;
  SET_P_LOW_FULL_OFF;
}
// Transition tools
// Sets up the deadtime timer
void NBCStartDeadtimeTimer()
{
  noInterrupts();
  // Set TCNT2 = 0 to start from the bottom
  TCNT2 = 0;
  OCR2A = NBC_DeadtimeValue;
  TIMSK2 = (1 << TOIE2); // Turn on Overflow Interrupt
  interrupts();
}
bool NBCRunPusher()
{
  if( NBC_PusherLocked )
  {
    return false;
  }

  NBC_PusherLocked = true;
  
  NBC_DeadtimeValue = DEADTIME_BRAKE_TO_RUN;
  
  SET_P_HIGH_FULL_OFF;
  NBC_DeadtimeTransition = TRANSITION_RUN;
  NBCStartDeadtimeTimer();
  return true;
}
// Starts stopping the pusher
bool NBCStopPusher()
{
  if( NBC_PusherLocked )
  {
    return false;;
  }

  NBC_PusherLocked = true;
  
  NBC_DeadtimeValue = DEADTIME_RUN_TO_BRAKE;
  SET_P_LOW_FULL_OFF;
  NBC_DeadtimeTransition = TRANSITION_BRAKE;
  NBCStartDeadtimeTimer();
  return true;
}
// Starts idling the pusher
bool NBCIdlePusher()
{
  if( NBC_PusherLocked )
  {
    return false;
  }

  NBC_PusherLocked = true;
  
  NBC_DeadtimeValue = max( DEADTIME_RUN_TO_BRAKE, DEADTIME_BRAKE_TO_RUN);
  SET_P_LOW_FULL_OFF;
  SET_P_HIGH_FULL_OFF;
  NBC_DeadtimeTransition = TRANSITION_IDLE;
  NBCStartDeadtimeTimer();

  return true;
}

bool FlyshotStartMotorsAndWait( unsigned long WaitTime )
{
  unsigned long StartTime = millis();
  FlyshotStartMotors();
  while( !FLYWHEELS_GOOD )
  {
    if( millis() - StartTime > WaitTime )
    {
      FlyshotStopMotors();
      return false;
    }
    NBCProcessFlywheelSpeed();
  }

  return true; // Motors started
}

bool FlyshotStopMotorsAndWait( unsigned long WaitTime )
{
  unsigned long StopTime = millis();
  FlyshotStopMotors();
  while( (NBC_Tach1RunningSpeed > 0) || (NBC_Tach2RunningSpeed > 0) )
  {
    if( millis() - StopTime > WaitTime )
    {
      return false;
    }
    NBCProcessFlywheelSpeed();
  }

  return true; // Motors started
}

float NBCGetVoltage()
{
  float Sample = analogRead( PIN_BATTERY );
  return ((Sample * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0));  // Voltage dividor - 47k and 10k
}

void NBCWait( unsigned long WaitTime )
{
  unsigned long StartTime = millis();
  while( millis() - StartTime < WaitTime )
  {
    NBCProcessFlywheelSpeed();
  }  
}

#endif
