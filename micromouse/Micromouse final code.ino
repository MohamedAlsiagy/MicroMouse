

//modes to see which way the car should go
#define GO_BACK 0
#define END_MAZE 1
#define GO_RIGHT 2
#define GO_LEFT 3
#define GO_STRAIGHT 4
#define FINISHED 5
#define GO_STRIAGHTPID 6

//pins for sensors

const int trigPin1 = PA8;
const int echoPin1 = PA11;
const int trigPin2 = PB13;
const int echoPin2 = PB12;
const int trigPin3 = PB10;
const int echoPin3 = PB11;

//pins for buttons
#define buttonPin 1 //to be modified for the dip switches


//Initializing variables for sensors
long duration, distance, RightSensor, FrontSensor, LeftSensor;


int previous_error = 0;
int previous_error2 = 0;
int previous_error3 = 0;
int previous_error4 = 0;
int previous_error5 = 0;

//Pins for Motor
#define MotorPinA PB9 // for motor A direction, high is CW and low is CCW
#define MotorSpeedPinA PB1 // Pin that controls speed for motor A
#define MotorBrakePinA PA7 // Pin that controls brake for motor A
#define MotorPinB PB7  // for motor B direction, high is CW and low is CCW
#define MotorSpeedPinB PB0// Pin that controls speed for motor B
#define MotorBrakePinB PB8// Pin that controls speed for motor B
#define DipButton1 PB6
#define DipButton2 PB5
#define DipButton3 PB4
#define DipButton4 PB3


// button states for DIP
int buttonstateDIP1 = 0, buttonstateDIP2 = 0, buttonstateDIP3 = 0, buttonstateDIP4 = 0;

// pin for the compass
#define CurrentAnglePin PA5 // Pin for the compass angle

//angle variable

int currentAngle;
int startingAngle = 4; /////////////////

//Turning variables
#define CW  HIGH
#define CCW LOW


//constants
#define runExtraInchConst 2000
#define runExtraInchConst2 1000
#define turnLeft 630
#define turnRight 580
#define turnBack 1131

int pLength = 0;
int pIndex = 0;
int mode = 0;
int wallDistance = 10; //distance from wall to robot
int deviation = 15; ////////////////////////////////////////////////
int statuss = 0;
int L_enable_val = 255 ; //can be modified //////////////////////////
int R_enable_val = 255 ;
int posX = 1;
int posY = 1;
int orientation = 90 ; //can be modified

char path[60] = {'0'};

int buttonState = 0;
int stage = 0;



void setup() {
  //Sensors Setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  //Motors Setup

  // motor A pin assignment
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorSpeedPinA, OUTPUT);
  pinMode(MotorBrakePinA, OUTPUT);

  // motor B pin assignment
  pinMode(MotorPinB, OUTPUT);
  pinMode(MotorSpeedPinB, OUTPUT);
  pinMode(MotorBrakePinB, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(CurrentAnglePin, INPUT);


  //current angle setup
  delay(2600);
  currentAngle = map(analogRead(CurrentAnglePin) , 0 , 4095 , 0 , 360);
  startingAngle = currentAngle;



  //DIP switch code
  pinMode(DipButton1, INPUT_PULLUP);
  pinMode(DipButton2, INPUT_PULLUP);
  pinMode(DipButton3, INPUT_PULLUP);
  pinMode(DipButton4, INPUT_PULLUP);


  buttonstateDIP1 = digitalRead(DipButton1);
  buttonstateDIP2 = digitalRead(DipButton2);
  buttonstateDIP3 = digitalRead(DipButton3);
  buttonstateDIP4 = digitalRead(DipButton4);

  Serial.begin(115200);

}

void loop()
{
  //
  //  while (buttonstateDIP3 == 0) {
  //    Serial.println("Button 3 activated");
  //    mazeOptimization();
  //
  //  }
  //
  //  while (buttonstateDIP4 == 0 && buttonstateDIP1 == 0 ) {
  //    addPath( 'S');
  //    addPath( 'S');
  //    mazeOptimization();
  //    mazeSolve2();
  //
  //  }
  //  while (buttonstateDIP4 == 0 && buttonstateDIP2 == 0) {
  //    addPath( 'S');
  //    addPath( 'L');
  //    mazeOptimization();
  //    mazeSolve2();
  //
  //  }
  //  while (buttonstateDIP1 == 0 || buttonstateDIP2 == 0) {
  //    mazeSolve2();
  //  }
  mazeSolve2();
  // addPath('L');
  // addPath('R');
  // addPath('S');
  // mazeOptimization();


}

void mazeSolve2(void) {
  // solving = 0; reach Maze End = 1

  while (readSensors2() == 1)
  {
    goStraight();
  }

  mazeSolve();
}
void mazeSolve(void)
{
  digitalWrite(MotorPinA , LOW);
  digitalWrite(MotorBrakePinA , LOW);
  digitalWrite(MotorPinB , LOW);
  digitalWrite(MotorBrakePinB , LOW);
  delay(550);

  mode = readSensors();
  Serial.print(LeftSensor);
  Serial.print(" - ");
  Serial.print(FrontSensor);
  Serial.print(" - ");
  Serial.println(RightSensor);
  switch (mode)
  {
    case GO_BACK:
      goAndTurn (180);
      addPath('B');
      break;

    case GO_RIGHT:
      goAndTurn (270);
      runExtraInch();
      addPath('R');
      break;

    case GO_LEFT:

      goAndTurn (90);
      runExtraInch();
      addPath('L');
      break;
    case GO_STRIAGHTPID:
      while (readSensors2() == 1)
      {
        goStraight();
      }
      break;

    case GO_STRAIGHT:
      runExtraInch();
      break;

    case FINISHED:
      digitalWrite(MotorPinA , LOW);
      digitalWrite(MotorBrakePinA , LOW);
      digitalWrite(MotorPinB , LOW);
      digitalWrite(MotorBrakePinB , LOW);

      statuss = 1;
      break;


  }


}
void goAndTurn(int degrees)
{
  L_enable_val = 255;
  R_enable_val = 255;
  if (degrees == 90)
  {


    digitalWrite(MotorPinA , HIGH);
    digitalWrite(MotorBrakePinA , LOW);
    digitalWrite(MotorPinB , LOW);
    digitalWrite(MotorBrakePinB , HIGH);

    analogWrite(MotorSpeedPinA , R_enable_val);
    analogWrite(MotorSpeedPinB , L_enable_val);

    //pidCompass(90);
    delay(turnLeft);
    digitalWrite(MotorPinA , LOW);
    digitalWrite(MotorBrakePinA , LOW);
    digitalWrite(MotorPinB , LOW);
    digitalWrite(MotorBrakePinB , LOW);
    delay(400);
    orientation += 90;
  }
  else if (degrees == 180)
  {

    digitalWrite(MotorPinA , HIGH);
    digitalWrite(MotorBrakePinA , LOW);
    digitalWrite(MotorPinB , LOW);
    digitalWrite(MotorBrakePinB , HIGH);

    analogWrite(MotorSpeedPinA , R_enable_val);
    analogWrite(MotorSpeedPinB , L_enable_val);

    //pidCompass(180);
    delay(turnBack);
    digitalWrite(MotorPinA , LOW);
    digitalWrite(MotorBrakePinA , LOW);
    digitalWrite(MotorPinB , LOW);
    digitalWrite(MotorBrakePinB , LOW);
    delay(400);
    orientation += 180;
  }
  else if (degrees == 270)
  {

    digitalWrite(MotorPinA , LOW);
    digitalWrite(MotorBrakePinA , HIGH);
    digitalWrite(MotorPinB , HIGH);
    digitalWrite(MotorBrakePinB , LOW);

    analogWrite(MotorSpeedPinA , R_enable_val);
    analogWrite(MotorSpeedPinB , L_enable_val);

    // pidCompass(270);
    delay(turnRight);
    digitalWrite(MotorPinA , LOW);
    digitalWrite(MotorBrakePinA , LOW);
    digitalWrite(MotorPinB , LOW);
    digitalWrite(MotorBrakePinB , LOW);
    delay(400);
    orientation += 270;
  }
  orientation = orientation % 360;
  previous_error = 0;
  previous_error2 = 0;
  previous_error3 = 0;
  previous_error4 = 0;
  previous_error5 = 0;
  R_enable_val = 255;
  L_enable_val = 255;
}

void goStraight(void)
{

  digitalWrite(MotorPinA , HIGH);
  digitalWrite(MotorBrakePinA , LOW);
  digitalWrite(MotorPinB , HIGH);
  digitalWrite(MotorBrakePinB , LOW);

  analogWrite(MotorSpeedPinA , R_enable_val);
  analogWrite(MotorSpeedPinB , L_enable_val);



  drive_straight(RightSensor , LeftSensor);
  Serial.print(" straight ");
}


void drive_straight( int left90_sensor,  int right90_sensor) // 2 or 4 sensors?
{

  // checking if the robot is already in striaght line


  // analog read values of 2 or 4 sensors passed to function
  static int Kp = 64, Ki = 16, Kd = 8 , KO = 4 ;     // constants for scaling P I D effects (will need adjusting)
  static int error, P, I = 0,  D , o;      // error variables
  int total;
  Serial.println(I);
  error = right90_sensor - left90_sensor ;



  P = error * Kp;

  I = (previous_error + previous_error2 + previous_error3 + previous_error4 + previous_error5 + error) * Ki;

  D = (error - previous_error) * Kd;

  if ( error <= 0 ) {
    o = - ( 1 / (abs(error) + 1)) * D * KO * pow((right90_sensor - 10) * (left90_sensor - 10) , 0.5);
  }
  else {
    o = ( 1 / (abs(error) + 1)) * D * KO * pow((right90_sensor - 10) * (left90_sensor - 10) , 0.5) ;
  }

  // may take out
  previous_error = error;
  previous_error2 = previous_error;
  previous_error3 = previous_error2;
  previous_error4 = previous_error3;
  previous_error5 = previous_error4;

  total = (P + I + D) ; // +o
  Serial.print(total);
  {
    L_enable_val -= (total) / 8;
    L_enable_val = constrain(L_enable_val, 120, 255);      // may need to adjust

    R_enable_val += (total) / 8 ;
    R_enable_val = constrain(R_enable_val, 120 , 255);

    analogWrite(MotorSpeedPinA, R_enable_val);            // enable pins and values
    // must be global
    analogWrite(MotorSpeedPinB, L_enable_val);          // arduino uses analogWrite
  }

}

void pidCompass(int angle) {
  // analog read values of 2 or 4 sensors passed to function

  static int Kp = 6 ;     // constants for scaling P I D effects (will need adjusting)
  static int P, I = 0,  D , o;      // error variables
  int error = 100;
  int total;
  Serial.println(I); //////////////////////////////ORIENTATION add

  while (error >= 10) {
    currentAngle = map(analogRead(CurrentAnglePin) , 0 , 4095 , 0 , 360) ;
    Serial.println(currentAngle);
    error = currentAngle  - ((orientation + angle + startingAngle) % 360) ;
    // Correct for when signs are reversed.
    if (error < 0) {
      error += 360;
    }

    // Check for wrap due to addition of declination.
    if (error > 185) {
      error -= 360;
    }

    error = abs(error);

    P = error * Kp;

    I = 0;

    D = 0;

    // may take out
    previous_error = error;
    previous_error2 = previous_error;
    previous_error3 = previous_error2;
    previous_error4 = previous_error3;
    previous_error5 = previous_error4;

    total = (P + I + D) ;
    Serial.print(total);
    {
      L_enable_val = total;
      L_enable_val = constrain(L_enable_val, 0, 255);      // may need to adjust

      R_enable_val = total ;
      R_enable_val = constrain(R_enable_val, 0 , 255);
    }
  }
}

void runExtraInch(void)
{
  //  int front1  = 0;
  //  int front2 = 700;
  //  if (orientation == 90)
  //  {
  //    posY += 1;
  //  }
  //  else if (orientation == 180)
  //  {
  //    posX -= 1;
  //  }
  //  else if (orientation == 270)
  //  {
  //    posY -= 1;
  //  }
  //  else if (orientation == 360 || orientation == 0)
  //  {
  //    posX += 1;
  //  }

  //  SonarSensor(trigPin2, echoPin2);
  //  front1 = distance;

  digitalWrite(MotorPinA , HIGH);
  digitalWrite(MotorBrakePinA , LOW);
  digitalWrite(MotorPinB , HIGH);
  digitalWrite(MotorBrakePinB , LOW);

  analogWrite(MotorSpeedPinA , 255);
  analogWrite(MotorSpeedPinB , 255);

  // while (front1 - front2 < 30)
  //  {
  //    SonarSensor(trigPin2, echoPin2);
  //    front2 = distance;
  //    delay(150);
  //  }
  delay(1533);

  digitalWrite(MotorPinA , LOW);
  digitalWrite(MotorBrakePinA , LOW);
  digitalWrite(MotorPinB , LOW);
  digitalWrite(MotorBrakePinB , LOW);

  delay(600);


}
//void runExtraInch2(void)
//{
//  /**
//    brake('B', 1);
//    brake('B', 0);
//    brake('A', 1);
//    brake('A', 0);
//
//    moveMotor('B', CW, 255);
//    moveMotor('A', CCW, 255);
//    delay(runExtraInchConst2);
//    brake('B', 1);
//    brake('A', 1);
//  */
//}

//telling where to turn
void mazeTurn (char dir)
{
  switch (dir)
  {
    case 'L': // Turn Left
      goAndTurn (90);
      break;

    case 'R': // Turn Right
      goAndTurn (270);
      break;

    case 'B': // Turn Back
      goAndTurn (180);
      break;

    case 'S': // Go Straight
      //runExtraInch();
      break;
  }
}
/**
   add Path and simplify path is inspired by an online guide and modified to our needs

 * */
//stores the path travelled and calls on simplify
void addPath(char direction)
{
  path[pLength] = direction; // Store the intersection in the path variable.
  pLength++;
  simplifyPath(); // Simplify the learned path.
}
//simplifies the path
void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (pLength < 3 || path[pLength - 2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[pLength - i])
    {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle)
  {
    case 0:
      path[pLength - 3] = 'S';
      break;
    case 90:
      path[pLength - 3] = 'R';
      break;
    case 180:
      path[pLength - 3] = 'B';
      break;
    case 270:
      path[pLength - 3] = 'L';
      break;
  }

  // The path has reduced by 2
  pLength -= 2;

}

void mazeOptimization (void)
{
  for (int i = 0; i < pLength; 
  {
    while (readSensors2() == 1)
    {
      goStraight();
      // goStraightNoStop(); //adding a delay could be used to make sure the robot is in the middle of the cell
      // delay(?????);
      // brake('B', 1);
      // brake('A', 1);

    }
    mazeTurn(path[i]);
    runExtraInch();
  }
}
//check if path is straight
int readSensors2(void)
{
  SonarSensor(trigPin1, echoPin1);
  LeftSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  FrontSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  RightSensor = distance;
  if (LeftSensor <= (wallDistance + deviation))
  {
    if ( RightSensor <= (wallDistance + deviation))
    {
      if (FrontSensor >= (8))
      {
        //delay
        return 1;
      }
    }
  }
  return 0;
}
//check how car should turn
int readSensors (void)
{
  SonarSensor(trigPin1, echoPin1);
  LeftSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  FrontSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  RightSensor = distance;

  //  if ((posY == 7 || posY == 6) && (posX == 7 || posX == 6 ) )
  //  {
  //    return FINISHED;
  //  }
  if (buttonstateDIP1 == 0) {
    Serial.println("Button 1 activated");
    if (LeftSensor <= (wallDistance + deviation))
    {
      Serial.println("L closed");
      if ( RightSensor <= (wallDistance + deviation))
      {
        Serial.println("R closed");

        if (FrontSensor <= (wallDistance + 10))
        {
          Serial.println("F closed");
          return GO_BACK;
        }
        else
        {
          Serial.println("F open and all else closed");
          return GO_STRIAGHTPID;
        }
      }
      else if (FrontSensor <= (wallDistance + 10))
      {
        Serial.println("R only open ");
        return GO_RIGHT;
      }
      else
      {
        return GO_STRAIGHT;
        Serial.println("L closed but s open ");
        addPath('S');
      }
    }
    else
    {
      return GO_LEFT;
    }
  }
  else if (buttonstateDIP2 == 0) {
    Serial.println("Button 2 activated");
    if (RightSensor <= (wallDistance + deviation))
    {
      Serial.println("R closed");
      if ( LeftSensor <= (wallDistance + deviation))
      {
        Serial.println("L closed");

        if (FrontSensor <= (wallDistance + 10))
        {
          Serial.println("F closed");
          return GO_BACK;
        }
        else
        {
          Serial.println("F open and all else closed");
          return GO_STRAIGHT;
        }
      }
      else if (FrontSensor <= (wallDistance + 10))
      {
        Serial.println("L only open ");
        return GO_LEFT;
      }
      else
      {
        return GO_STRAIGHT;
        Serial.println("L closed but s open ");
        addPath('S');
      }
    }
    else
    {
      return GO_RIGHT;
    }
  }

}
void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = constrain(duration * 0.034 / 2 , 4 , 120); // modified to match sharp ir

  if (echoPin == PA11) {
    Serial.print("Left distance  ");
  }
  else if (echoPin == PB12) {
    Serial.print("Front distance  ");
  }
  else {
    Serial.print("Right distance  ");
  }
  Serial.println(distance);
}

/*
  @param motor is char A or B refering to motor A or B.
  @param dir is motor direction, CW or CCW
  @speed is PWM value between 0 to 255
*/

void moveMotor(char motor, int dir, int speed)
{
  int motorPin;
  int motorSpeedPin;

  if (motor == 'A')
  {
    motorPin      = MotorPinA;
    motorSpeedPin = MotorSpeedPinA;
  }
  else
  {
    motorPin      = MotorPinB;
    motorSpeedPin = MotorSpeedPinB;
  }
  digitalWrite(motorPin, dir);// set direction for motor
  analogWrite(motorSpeedPin, speed);// set speed of motor
}//moveMotor end

/*
   brake, stops the motor, or releases the brake
   @param motor is character A or B
   @param brk if  1 brake, if 0, release brake
   example of usage:
   brake('A', 1);// applies brake to motor A
   brake('A', 0);// releases brake from motor A
*/
