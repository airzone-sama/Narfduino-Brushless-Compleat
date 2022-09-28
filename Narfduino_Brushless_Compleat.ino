#include "NBC.h"

#define PIN_TRIGGER 6

#define MIN_BATTERY_VOLTAGE (3.0 * 4)
#define MAX_WAIT_TIME 1000
#define SOLENOID_PULSE_TIME 50
#define SOLENOID_RETRACT_TIME 40

void setup() 
{
  pinMode( PIN_TRIGGER, INPUT_PULLUP );

  delay( 1000 );

  NBCInit();

  delay( 1000 );

  FlyshotSetNewMotorSpeed( 5000 );
  delay( 500 );
  FlyshotSetMotorDirectionForward( FLYSHOT_ESC_A );
  delay( 500 );
  FlyshotSetMotorDirectionForward( FLYSHOT_ESC_B );
  delay( 500 );
  FlyshotBeep1( FLYSHOT_ESC_BOTH );
  delay( 1000 );

  CalibrateFlywheels();
}

void loop() 
{
  if( NBCGetVoltage() < MIN_BATTERY_VOLTAGE )
    return;

  while( digitalRead( PIN_TRIGGER ) == LOW )
  {
    if( !FlyshotStartMotorsAndWait( MAX_WAIT_TIME ) )
    {
      FlyshotStopMotorsAndWait( MAX_WAIT_TIME );
      FlyshotBeep2( FLYSHOT_ESC_BOTH );
      NBCWait( 5000 );
      return;
    }

    NBCSolenoidOn();
    NBCWait( SOLENOID_PULSE_TIME );
    NBCSolenoidOff();
    NBCWait( SOLENOID_RETRACT_TIME );    
    NBCProcessFlywheelSpeed();
  }

  NBCProcessFlywheelSpeed();

  FlyshotStopMotors();
}
