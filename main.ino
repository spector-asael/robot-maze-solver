#include "MeMegaPi.h"

// Motors
MeMegaPiDCMotor motorL(PORT4B); 
MeMegaPiDCMotor motorR(PORT3B);

// Ultrasonic sensor is on the right-side of the robot
MeUltrasonicSensor ultraRight(PORT_7); 

// Line follower is on the front of the robot
MeLineFollower lineFront(PORT_8);  

// LED light that lights up upon completion of the maze.  
MeRGBLed rgbLed(PORT_6);

// The base speed for the motors used throughout the program.
const int BASESPEED = 60;

// This flag indicates whether or not the robot has entered "island mode" 
// Island mode is achieved once the robot does 4 consecutive left turns.
bool insideMode = false;

// This flag is used to ensure that the robot only performs the "turn around" maneuver once, after it detects 4 consecutive left turns.
// The robot turns around when it enters "island mode" to ensure that it is facing the correct direction to navigate the island.
bool turned = false;

// Flag to check if the maze has been fully completed
bool completed = false;

// This counter keeps track of how many consecutive left turns the robot has made.
int count = 0;

/*
            HELPER FUNCTIONS
*/

// Simply stops both motors by setting their speed to 0.
void stopMotors() {
  motorL.run(0);
  motorR.run(0);
}

// Moves the robot forward. 
// This function executues whenever the robot is perfectly aligned against the wall.
// The motor speeds are slightly faster for faster speeds.
void moveForward() {
  Serial.println("[MOTOR] Forward");
  motorL.run(BASESPEED + 20);
  motorR.run(-BASESPEED - 20);
}

// The turn left function. 
// The robot does a very harsh left turn for faster speed.
void turnLeft() {
  Serial.println("[MOTOR] TURN LEFT");
  // First we make sure the robot halts completely for a split second.
  stopMotors();
  delay(50);

  // Then we have it go back a little
  // To ensure that it has enough space to turn without colliding with the wall in front.
  motorL.run(-BASESPEED);
  motorR.run(BASESPEED);
  delay(110);


  // Once it's turned around, we stop it to a hault for a split second again.
  stopMotors();
  delay(50);
  
  // Now the robot faces left at the speed of sound!
  // Look at it go!
  motorL.run(-BASESPEED * 4);
  motorR.run(-BASESPEED * 4);
  delay(200);

  // Now we can stop the motor and end the function.
  stopMotors();
  delay(150);
}

// The move right function.
// The robot doesn't do a hars right turn like left turns
// The reason for this being that right turns are detected via the ultrasonic sensor
// So it can perform right turns pre-emptively before its hit a wall
void moveRight() {
    Serial.println("[MOTOR] MOVE LEFT");
    motorL.run(BASESPEED * 2);
    motorR.run(-BASESPEED - 20);
}

// The turn around function.
// Whenever the robot hits island mode, it turns around. 
// When the robot turns executes its 4th left turn, it enters into island mode
// So for it to do a 180, it must do another left turn as it just finished...
// ...completing a left turn.
void turnAround() {
  turnLeft();
  readjustRightwardsVeryStrong();
  delay(800);
  moveForward();
  delay(100);
}

// A very slight left adjustment for when the robot is slightly too close to the wall.
void readjustLeftwards() {
  Serial.println("[MOTOR] READJUST RIGHT");
  motorL.run(BASESPEED);
  motorR.run(-BASESPEED - 20);
}

// A harsh right adjustment for when the robot is straying too far away from the wall!
void readjustLeftwardsStrong() {
  Serial.println("[MOTOR] READJUST RIGHT STRONG");
  motorL.run(BASESPEED );
  motorR.run(-BASESPEED - 40);
}

// A slight right adjustment. This is used when the robot is slightly too far away from the wall.
void readjustRightwards() {
  Serial.println("[MOTOR] READJUST LEFT");
  motorL.run(BASESPEED + 20);
  motorR.run(-BASESPEED);
}

// A right adjustment for when the robot is straying too far away from to the wall.
void readjustRightwardsStrong() {
  Serial.println("[MOTOR] READJUST RIGHT STRONG");
  motorL.run(BASESPEED + 35);
  motorR.run(-BASESPEED);
}

// A very harsh right adjustment for when the robot is straying very far away from the wall.
// The robot slows down the left motor and speeds up the 
// right motor to make a very sharp right turn to get back on track.
void readjustRightwardsVeryStrong() {
  Serial.println("[MOTOR] READJUST LEFT VERY STRONG");
  motorL.run(BASESPEED + 30);
  motorR.run(-BASESPEED - 10);
}

//                SETUP

void setup() {
  // Sets the colour to off at the start of the program.
  // The RGB LED is used to indicate when the robot has entered island mode.
    rgbLed.setColor(0, 0, 0, 0);
    rgbLed.show();
    Serial.begin(115200);
    Serial.println("===== SYSTEM START =====");
    delay(100);
}

//                 LOOP 

void loop() {
  if (completed) {
    // If the maze has been completed, we don't want to execute any more code in the loop function, 
    // as we have already completed the maze and we don't want to do anything else.
    stopMotors();
    return;
  }

  // Whenever the robot detects 4 consecutive left turns, it enters island mode.
  if (count == 4 && !turned) {
    turnAround();
    turned = true; // We set the turned flag to true to ensure 
                  // that we don't execute the turn around maneuver again.
    insideMode = true;
    moveForward(); 
    // We move forward a little bit to ensure that we have fully entered the 
    // island before we start navigating it.
    // We don't want to immediately detect the turn to our right 
    delay(1000);
  }
  
  // We read the linefront to check what state its currently in every loop iteration.
  int front = lineFront.readSensors();
  // We read the ultrasonic sensor to check how far we are from the wall on the right every loop iteration.
  int rightDist = ultraRight.distanceCm();
  // We check if there is a wall on the right by checking if the distance is between 0 and 15.
  bool rightBlocked = (rightDist > 2 && rightDist < 15);

  // If we are not in island mode, we execute the basic movement function to navigate the maze.
  // If we are in island mode, we execute the island movement function to navigate the island.
  if (!insideMode) {
    basicMovement(front, rightDist, rightBlocked);
  } else {
    islandMovement(front, rightDist, rightBlocked);
  }
}

//                 MAIN LOGIC

// Whenever the robot is not in island mode, it executes this function to navigate the maze.
void basicMovement(int front, int rightDist, bool rightBlocked) {
  // Whenever a wall has been hit, it means that we went straight without finding a right opening.
  // We must turn left to find the next path.
    if (front != 0) {
        stopMotors();
        count++; // We also increase the left turn counter as we have done a left turn.
        turnLeft();
        return;
    }
  // Whenever there is an opening on the right, we want to take it 
  // immediately without hesitation!
    if (!rightBlocked) {
        count = 0; 
        // We reset the left turn counter as we have found a right opening and are taking it, 
        // so we are not doing consecutive left turns.
        moveRight();
        return;
    }

  // Correcting the robot's path to ensure that it stays parallel to the wall on the right.
  if (rightDist <= 3.5 && rightDist == 400) 
  // Way too close to the wall! We need to make a harsh left turn to readjust ourselves.
    readjustLeftwardsStrong();
  else if (rightDist <= 4.5)
  // Slightly too close to the wall. We need to make a slight left turn to readjust ourselves.
    readjustLeftwards();
  else if (rightDist >= 5.5 && rightDist <= 7)
  // Slightly too far away from the wall. We need to make a slight right turn to readjust ourselves.
    readjustRightwards();
  else if (rightDist > 7 && rightDist <= 8.5)
  // Way too far away from the wall! We need to make a harsh right turn to readjust ourselves.
    readjustRightwardsStrong();
  else if (rightDist > 8.5 && rightDist <= 15)
  // Way too far away from the wall! We need to make a very harsh right turn to readjust ourselves.
    readjustRightwardsVeryStrong();
  else
    moveForward(); // only executes at 4.5 - 4.6cms away from the wall
}

// Once the robot enters island mode, it executes this function to navigate the island.
void islandMovement(int front, int rightDist, bool rightBlocked) {
  // Whenever the robot hits a wall in front of it,
  // It means that we have found the center!
  // The robot can only find a wall if it has entered the center of the island, 
  // as the walls of the island itself won't ever be reached 
  // Due to the fact we go in rightwards circles around the island.
    if (front != 0) {
        stopMotors();
        rgbLed.setColor(0, 0, 255, 0); 
        rgbLed.show();
        completed = true; // We set the completed flag to true to indicate that we have completed the maze.
        Serial.println("===== MAZE COMPLETED =====");
    }

  // Whenever there is an opening on the right, we want to take it 
  // immediately without hesitation!
    if (!rightBlocked) {
        count = 0;
        moveRight();
        return;
    }

  // Correcting the robot's path to ensure that it stays parallel to the wall on the right.
  if (rightDist <= 3.5 && rightDist == 400) 
  // Way too close to the wall! We need to make a harsh left turn to readjust ourselves.
    readjustLeftwardsStrong();
  else if (rightDist <= 4.5)
  // Slightly too close to the wall. We need to make a slight left turn to readjust ourselves.
    readjustLeftwards();
  else if (rightDist > 5.5 && rightDist <= 7)
  // Slightly too far away from the wall. We need to make a slight right turn to readjust ourselves.
    readjustRightwards();
  else if (rightDist > 7 && rightDist <= 8.5)
  // Way too far away from the wall! We need to make a harsh right turn to readjust ourselves.
    readjustRightwardsStrong();
  else if (rightDist > 8.5 && rightDist <= 15)
  // Way too far away from the wall! We need to make a very harsh right turn to readjust ourselves.
    readjustRightwardsVeryStrong();
  else
    moveForward();
}