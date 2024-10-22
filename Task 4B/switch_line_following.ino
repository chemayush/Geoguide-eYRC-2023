// IR sensor pins
const int extremeLeftSensor = 27;
const int leftSensor = 26;
const int middleSensor = 25;
const int rightSensor = 32;
const int extremeRightSensor = 35;

/* 
    Sensor array configuration (Front View):
    _____________________________________
    |ER|          |R||M||L|          |EL|         

*/
int previousMillis, currentMillis;

// Motor Pins 
const int lb = 18;            // Left Backward
const int lf = 19;            // Left Forward
const int lpwm = 5;           // Left PWM

const int rb = 22;            // Right Backward
const int rf = 21;            // Right Forward
const int rpwm = 23;          // Right PWM

const int led = 2;            // Green LED 
const int buzzer = 15;        // Buzzer

int elv, lv, mv, rv, erv;     // Sensor Readings

int ls = 234;                 // Left Motor Speed
int rs = 226;                 // Right Motor Speed (It is lesser than rs because right motor was turning a bit faster than left)

enum LineFollowingState {

  MIDDLE_LINE,
  EXTREME_LINE,

  RIGHT,
  LEFT

};

LineFollowingState state = MIDDLE_LINE;       // State of the line following depending on the type of road.
LineFollowingState els = LEFT;                // State to determine if it is on right side of the road or left side.

int node = 0;                                 

// ******************************************************* All the functions used ********************************************************

void extremeLineFollowing();      // Line following for roads of type 1.
void middleLineFollowing();       // Line following for roads of type-2.
void switchFollowing();           // Switches between middleLineFollowing() and extremeLineFollowing().
void updateState();               // updates the state of line following, i.e. the type of road being followed right now.
void nodeRoutine();               // Function executed when a node is detected.
void checkNode();                 // Checks which node it is on right now.
bool turnExists();                // Checks if a turn is to be taken on the current node or not.
void getValues();                 // Updates the values of ir sensors.

// NOTE: The speeds in the usage of these functions have been optimally chosen based on arena and our Vanguard Robot's form factor.

void right(int l, int r);         // Vanguard moves right.
void left(int l, int r);          // Vanguard moves leftt.
void moveForward();               // Vanguard moves forward.
void moveBackward();              // Vanguard moves backwards.
void stop();                      // Vanguard stops.
void nonBlockingDelay(int intv);  // A non blocking delay. A replacement for traditional delay() function.

// ***************************************************************************************************************************************

// Setup function
// Initialising all the pins and waiting for 5000ms before the run starts.

void setup() {

  pinMode(extremeLeftSensor, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(middleSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(extremeRightSensor, INPUT);

  pinMode(lf, OUTPUT);
  pinMode(lb, OUTPUT);
  pinMode(lpwm, OUTPUT);

  pinMode(rf, OUTPUT);
  pinMode(rb, OUTPUT);
  pinMode(rpwm, OUTPUT);

  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Serial.begin(9600);

  digitalWrite(buzzer, HIGH);
  digitalWrite(led, LOW);

  nonBlockingDelay(5000);

  digitalWrite(buzzer, LOW);
  digitalWrite(led, HIGH);

  nonBlockingDelay(1000);

  digitalWrite(buzzer, HIGH);
  digitalWrite(led, LOW);

}

// *********************************************************************

/* 
    Gets the current readings of the sensors and then updates the state 
    according to the new values. It then chooses the appropriate Line Following 
    algorithm accorrding to current state. 
*/

void loop() {

  getValues();          
  updateState();
  switchFollowing();

}

// *********************************************************************

/* 
    Updates the 'state' based on sensor values.
*/

void updateState() {
  if (mv) {
    state = MIDDLE_LINE;
  } else if (state == EXTREME_LINE) {
    if ((els == LEFT || els == RIGHT) && (rv || lv)) {
      state = MIDDLE_LINE;
    }
  } else if (elv || erv) {
    if (lv || mv || rv) {
      state = MIDDLE_LINE;
    } else {
      state = EXTREME_LINE;
    }
  }
}

// *********************************************************************

/*
    Switches to suitable Line Following algorithm according to the 'state'
*/

void switchFollowing() {

  if (state == MIDDLE_LINE) {
    middleLineFollowing();
  }
  
  if (state == EXTREME_LINE) {
    extremeLineFollowing();
  }

}

// *********************************************************************

/*
    Line following algorithm for following the roads of type-1. It uses the
    extreme right and extreme left sensors to sense the black lines on the 
    extreme sides of the road. 

    Abbreviations:
    ER: Extreme Right
    R: Right
    L: Left
    EL: Extreme Left
    els (it's a variable): Extreme Line State

    -> If the ER sensor is '1', then it takes a slight left. 
       els = RIGHT
    -> If the EL sensor is '1', then it takes a slight right.
       els = LEFT
    -> If the R sensor is '1', that would mean that ER sensor wasn't 
       able to sense the black line, hence it takes a left until the ER
       sensor is on black line.
       els = RIGHT
    -> If the L sensor is '1', that would mean that EL sensor wasn't 
       able to sense the black line, hence it takes a right until the ER
       sensor is on black line.
       els = LEFT
*/

void extremeLineFollowing() {

  getValues();
  
  if (lv) {

    els = LEFT;

    while (!elv) {
      getValues();
      right(ls, 0);
    }

  } else if (rv) { 

    els = RIGHT;

    while (!erv) {
      getValues();
      left(0, rs);
    }

  } else if (!elv && !erv) {

    moveForward();

  } else if (elv) {

    els = LEFT;

    if (!lv) {
      right(ls, 0);
    } else if (lv) {
      while (!elv) {
        getValues();
        right(ls, 0);
      }
    }

  } else if (erv) {

    els = RIGHT;

    if (!rv) {
      left(0, rs);
    } else if (rv) { 
      while (!erv) {
        getValues();
        left(0, rs);
      }
    }
  }

}

// *********************************************************************

/* 
    Line following algorithm for following the roads of type-2. It uses the 
    Left, Middle and Right sensors to sense the line which runs through the 
    middle of the road.

    Abbreviations:
    LS: Left Sensor
    RS: Right Sensor
    MS: Middle Sensor

    -> If MS is on the line:
          -> If LS and RS are not on the line, then go straight.
          -> If RS is on the line and LS is not on the line, then
             it takes a slight right.
          -> If LS is on the line and RS is not on the line, then
             it takes a slight left.
          -> If LS, MS, RS are all '1' then a node is detected and
             it executes the nodeRoutine() function.
    -> If MS is not on the line:
          -> If LS is on the line and RS isn't, then it takes a hard left.
          -> If RS is on the line and LS isn't, then it takes a hard right.
          -> If LS and RS both aren't on the line:
              -> If last node detected was 11, it stops. Else it goes straight.
*/

void middleLineFollowing() {

  if(mv) {    

    if(!lv && !rv) {
      moveForward();
    } else if(rv && !lv) {
      right(ls, 140);
    } else if (lv && !rv) {
      left(130, rs);
    } else if(lv && rv) {
      nodeRoutine();
    }
  } else {
    if(lv && !rv) {
      left(100, ls*1.06);
    } else if(!lv && rv) {
      right(rs*1.05, 110);
    } else if (!lv && !rv) {
      if (node == 11) {
        stop();
        digitalWrite(buzzer, LOW);
        digitalWrite(led, HIGH);

        nonBlockingDelay(5000);

        digitalWrite(buzzer, HIGH);
        digitalWrite(led, LOW);
        while (1) {
          stop();
        }
      }
      moveForward();
    }
  }

}

// *********************************************************************

/*
    This function is executed when a node is encountered.
    
    -> 'node' variable is incremented.
    -> Turns on the LED and Buzzer for 1000ms
    -> Checks if there is a turn to be taken through the turnExists() function.
    -> If false, it it exits the node by going straight.
    -> Else, it executes the checkNode() function
*/

void nodeRoutine() {
  node++;

  digitalWrite(buzzer, LOW);
  digitalWrite(led, HIGH);

  nonBlockingDelay(1000);

  digitalWrite(buzzer, HIGH);
  digitalWrite(led, LOW);

  if (turnExists() == false) {  
    while (1) {
      getValues();
      if (lv && rv && mv) {
        moveForward();
      } else if (lv && !rv) {
        left(120, rs);
      } else if (!lv && rv) {
        right(ls, 120);
      } else if (!lv && !rv && mv) {
        break;
      }
    }
  } else {
    checkNode();
  }
}

// *********************************************************************

/*
    This function checks if there is a turn to be taken at the current node. 
    If the 'node' is 3, 4, 5, 6, 8 or 10 then it returns true, else false.
*/

bool turnExists() {

  if (node == 3 || node == 4 || node == 5 || node == 6 || node == 8 || node == 10) {
    return true;
  } else {
    return false;
  }

}

// *********************************************************************

/*
    This functions makes turns based on the values of 'node' variable.
    While taking turn, it checks if {lv = 0 and mv = 1 and rv = 0} configuration
    is reached. If it is found, it goes back to following the newly detected path.
*/

void checkNode() {

  if (node == 3) {
    do {
      getValues();
      right(ls*0.95, 0);

    } while (!(!lv && mv && !rv));

  } else if (node == 4) {
    do {
      getValues();
      left(0, rs*0.95);
    } while (!(!lv && mv && !rv));

  } else if (node == 5) {
    do {
      getValues();
      right(ls*0.95, 0);
    } while (!(!lv && mv && !rv));

  } else if (node == 6) {
    do {
      getValues();
      right(ls*0.95, 0);
    } while (!(!lv && mv && !rv));

  } else if (node == 8) {
    do {
      getValues();
      right(ls*0.95, 0);
    } while (!(!lv && mv && !rv));

  } else if (node == 10) {
    do {
      getValues();
      left(0, rs*0.95);
    } while (!(!lv && mv && !rv));

  } else {
    moveForward();
  }

}

// *********************************************************************

/*
    This function has been defined to avoid using the traditional delay()
    function, because when it is used, the CPU doesn't do anything for 
    that interval of time. This could make the system less responsive 
    as we depend entirely upon the sensor's response.
*/

void nonBlockingDelay(int intv) {

  previousMillis = millis();
  while (currentMillis - previousMillis < intv) {
    stop();
    currentMillis = millis();
  }

}

// *********************************************************************

void left(int l, int r) {
  analogWrite(lpwm, l);
  analogWrite(rpwm, r);

  digitalWrite(rb, LOW); 
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rf, HIGH);
  Serial.println("Left");
}

void right(int l, int r) {
  analogWrite(lpwm, l);
  analogWrite(rpwm, r);

  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rf, HIGH); 
  digitalWrite(rb, LOW);
  Serial.println("Right");
}

void moveForward() {
  analogWrite(lpwm, ls);
  analogWrite(rpwm, rs);

  Serial.println("move forward");
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rb, LOW);
  digitalWrite(rf, HIGH);
}

void moveBackward() {
  analogWrite(lpwm, ls);
  analogWrite(rpwm, rs);

  Serial.println("move backward");
  digitalWrite(lb, HIGH);
  digitalWrite(lf, LOW);
  digitalWrite(rb, HIGH);
  digitalWrite(rf, LOW);
}

void stop() {
  analogWrite(lpwm, ls);
  analogWrite(rpwm, rs);

  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  Serial.println("Stop");
}

void getValues() {
  elv = digitalRead(extremeLeftSensor);
  lv = digitalRead(leftSensor);
  mv = digitalRead(middleSensor);
  rv = digitalRead(rightSensor);
  erv = digitalRead(extremeRightSensor);
}

// *********************************************************************