/*
  Shark hand held vac using a subsumption architecture.
  
 */
#include <Servo.h>

#define LEDPIN  13
#define LEFTSW  10
#define RIGHTSW  9
#define TRIGPIN  7
#define ECHOPIN  8
#define SONARSERVO  4
#define LEFTSERVO   2
#define RIGHTSERVO  3

#define LEFT_FWD   leftServo.write(180);
#define RIGHT_FWD  rightServo.write(0);
#define LEFT_BACK  leftServo.write(0);
#define RIGHT_BACK rightServo.write(180);
#define LEFT_STOP  leftServo.write(90);
#define RIGHT_STOP rightServo.write(90);
#define SONAR_STOP sonarServo.write(90);

// Behaviors
#define C_NONE      0
#define C_STOP      1
#define C_FORWARD   2
#define C_REVERSE   3
#define C_TURNLEFT  4
#define C_TURNRIGHT 5

Servo sonarServo;
Servo leftServo;
Servo rightServo;

unsigned long duration;
unsigned long distance;
unsigned long revTime=0;
char servoDir = 0;
char state = 0;

// the setup routine runs once when you press reset:
void setup() 
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // make the pushbutton's pin an input:
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LEFTSW, INPUT_PULLUP);
  pinMode(RIGHTSW, INPUT_PULLUP);
  
  sonarServo.attach(SONARSERVO);
  leftServo.attach(LEFTSERVO);
  rightServo.attach(RIGHTSERVO);
}

// the loop routine runs over and over again forever:
void loop() 
{
  char command = C_NONE;
  
  // Arbitrate, highest priority tasks first
  //if ( command == C_NONE ) command = rest();
  if ( command == C_NONE ) command = detectSwitch();
  if ( command == C_NONE ) command = detectSonar();
  if ( command == C_NONE ) command = drive();

  motorOutput( command );   
}

void motorOutput( char command )
{
  switch ( command )
  {
    case C_NONE:                           break;
    case C_STOP:    LEFT_STOP; RIGHT_STOP; break;
    case C_FORWARD: LEFT_FWD; RIGHT_FWD;   break;
    case C_REVERSE: LEFT_BACK; RIGHT_BACK; break;
    case C_TURNLEFT: LEFT_BACK; RIGHT_FWD; break;
    case C_TURNRIGHT: LEFT_FWD; RIGHT_BACK; break;
  }
}

void fullStop()
{
  LEFT_STOP;
  RIGHT_STOP;
  
  sonarServo.detach();
  leftServo.detach();
  rightServo.detach();
}

// Rest for 1 minute out of every 3
char rest()
{
  static char state = 0;
  static unsigned long expireTime = millis() + 120000;
  char retval = C_NONE;

  switch (state)
  {
    case 0:
      if (millis() > expireTime)
      {
        expireTime = millis() + 60000;
        state++;
      }
      break;
      
    case 1:
      if (millis() > expireTime)
      {
        expireTime = millis() + 120000;
        state = 0;
      }
      retval = C_STOP;
      break;
  }
  
  return retval;
}

char detectSwitch()
{
  static char state = 0;
  static char dir = -1;
  char retval = C_NONE;
  
  switch (state)
  {
    case 0:
      if (digitalRead(LEFTSW) == HIGH)
      {
        dir = -1;
        state++;
      }
      else if (digitalRead(RIGHTSW) == HIGH)
      {
        dir = 1;
        state++;
      }
      else
        retval = C_NONE;
      break;
      
    case 1:
      retval = avoid(0);
      if ( retval == C_NONE )
      {
        state = 0;
      }
      break;
  }
  return retval;
}

char detectSonar()
{
  static char state = 0;
  static unsigned long expireTime = millis();
  static char dir = -1;  // values are -1, 0, 1
  char retval = C_NONE;
  
  switch( state )
  {
    case 0:  // point to next position
      digitalWrite(LEDPIN, HIGH);
      dir++;
      if ( dir > 1 )
      {
        dir = -1;
      }
        
      switch ( dir )
      {
        case -1: sonarServo.write(40); break;
        case  0: sonarServo.write(90); break;
        case  1: sonarServo.write(140); break;
      }
      expireTime = millis() + 1000;
      state++;
      break;
      
    case 1:  // wait for it to get there
      if ( millis() > expireTime )
        state++;
      break;
      
    case 2:  // Take a reading
      // pulse the trigger line
      digitalWrite(TRIGPIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(TRIGPIN, LOW);
      
      // wait for the echo
      duration = pulseIn(ECHOPIN, HIGH);
      
      distance = duration / 116;
      
      Serial.println( distance );
      
      if ( distance < 10 )
      {
        digitalWrite(LEDPIN, LOW);
        state++;
      }
      else   
        state = 0;
      break;
      
    case 3:  // avoid
      retval = avoid(0); //avoid(dir * -1);
      if ( retval == C_NONE )
      {
        state = 0;
        digitalWrite(LEDPIN, HIGH);
      }
      break;
  }
  
  /*
  Serial.print("left: ");
  Serial.print(digitalRead(LEFTSW));
  Serial.print("   right: ");
  Serial.println(digitalRead(RIGHTSW));
  delay(500);
  */
  return retval;
}

/*
  Implements a standard avoidance routine of:
    1. Back up for 3 seconds
    2. Turn for 3 seconds
  
  Direction of turn is:
    -1  - turn left
    0   - random direction
    1   - turn right  
    
  This routine counts on retval retaining its value from one call to the next
*/    
char avoid( char dir )
{
  static char state = 0;
  static unsigned long expireTime = millis();
  static char retval = C_NONE;
  
  switch (state)
  {
    case 0:
      expireTime = millis() + 500 + random(1,1500);
      state++;
      retval = C_REVERSE;
      break;
      
    case 1:
      if (millis() > expireTime)
      {
        expireTime = millis() + 500 + random(1,1500);
        state++;
        switch (dir)
        {
          case -1: retval = C_TURNLEFT; break;
          case  0: retval = random(C_TURNLEFT, C_TURNRIGHT + 1); break;
          case  1: retval = C_TURNRIGHT; break;
        }
      }
      break;
      
    case 2:
      if (millis() > expireTime)
      {
        state = 0;
        retval = C_NONE;
      }
      break;  
   }
  
  return retval;
}
/*
char avoidLeft()
{
  static char state = 0;
  static unsigned long expireTime;
  
  switch ( state )
  {
    case 0:
      LEFT_BACK;
      RIGHT_BACK;
      expireTime = millis() + 3000;
      state++;
      break;
    
    case 1:
      if (millis() > expireTime)
      {
        state = 0;
        return 0;
      }
      break;

    case 2:
      LEFT_FWD;
      RIGHT_BACK;
      expireTime = millis() + 3000;
      state++;
      break;
      
    case 3:
      if (millis() > expireTime)
      {
        state = 0;
        return 0;
      }
      break;
  }
  
  return 2;
}

char avoidRight()
{
  static char state = 0;
  static unsigned long expireTime;
  
  switch ( state )
  {
    case 0:
      LEFT_BACK;
      RIGHT_BACK;
      expireTime = millis() + 3000;
      state++;
      break;
    
    case 1:
      if (millis() > expireTime)
      {
        state = 0;
        return 0;
      }
      break;

    case 2:
      LEFT_BACK;
      RIGHT_FWD;
      expireTime = millis() + 3000;
      state++;
      break;
      
    case 3:
      if (millis() > expireTime)
      {
        state = 0;
        return 0;
      }
      break;
  }
  
  return 3;
}
*/
char drive()
{
  return C_FORWARD;
}


