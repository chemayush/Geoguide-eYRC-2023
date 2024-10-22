#include <WiFi.h>

const char* ssid = "ayush";
const char* password = "raspberrypi";
const char* host = "192.168.216.132";
const uint16_t port = 8080;

WiFiClient client;

const int extremeLeftSensor = 35;  //27
const int leftSensor = 32;         //26
const int middleSensor = 25;       //25
const int rightSensor = 26;        //32
const int extremeRightSensor = 27; //35

/* 
    Sensor array configuration (Front View):
    _____________________________________
    |ER|          |R||M||L|          |EL|         

*/
int previousMillis, currentMillis;

// Motor Pins 
const int rb = 18;            // Left Backward
const int rf = 19;            // Left Forward
const int rpwm = 5;           // Left PWM

const int lb = 22;            // Right Backward
const int lf = 21;            // Right Forward
const int lpwm = 23;          // Right PWM

const int led = 2;            // Green LED 
const int buzzer = 15;        // Buzzer

int elv, lv, mv, rv, erv;     // Sensor Readings

int ls = 171;                 // Left Motor Speed
int rs = 166;                 // Right Motor Speed (It is lesser than rs because right motor was turning a bit faster than left)

enum LineFollowingState {

  MIDDLE_LINE,
  EXTREME_LINE,

  RIGHT,
  LEFT

};

LineFollowingState state = MIDDLE_LINE;       // State of the line following depending on the type of road.
LineFollowingState els = LEFT;                // State to determine if it is on right side of the road or left side.

int node = 0;
int nodeCoords[12][2] = { {0, 0}, 
                               {0, 1}, {1, 1},
                               {0, 2}, {1, 2}, {2, 2},
                               {0, 3}, {1, 3}, {2, 3},
                               {0, 4}, {1, 4}, {2, 4},
                             };                           // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11   

int prevNode = 0;
int currNode = 0;
int nextNode = 1;             

String coordinatesString = "";
String pathString = "";
String eventString = "";

int path[100] = {};
char events[10] = {};

int i = 0;              //node iterator
int j = 0;              //events iterator
bool isStopped = false; //Vanguard is stopped at an event or not
float xpos, ypos;       //Position of the Vanguard

// ******************************************************* All the functions used ********************************************************

void extremeLineFollowing();                      // Line following for roads of type 1.
void middleLineFollowing();                       // Line following for roads of type-2.
void switchFollowing();                           // Switches between middleLineFollowing() and extremeLineFollowing().
void updateState();                               // updates the state of line following, i.e. the type of road being followed right now.
void nodeRoutine();                               // Function executed when a node is detected.
void checkNode1();                                // Checks which node it is on right now.
int crossProduct(int pn, int cn, int nn)          // 
void updateCoords();                              //
void getValues();                                 // Updates the values of ir sensors.
bool isUturn(int pn, int cn, int nn)              //
void parseAndPopulateArrayString(String input);   //
void parseAndPopulateArray(String input);         //

// NOTE: The speeds in the usage of these functions have been optimally chosen based on arena and our Vanguard Robot's form factor.

void right(int l, int r);         // Vanguard moves right.
void left(int l, int r);          // Vanguard moves leftt.
void moveForward();               // Vanguard moves forward.
void moveBackward();              // Vanguard moves backwards.
void stop();                      // Vanguard stops.
void uTurn();                     // Vanguard makes a U Turn.
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
  digitalWrite(buzzer, HIGH);

  Serial.begin(115200);

  digitalWrite(2, HIGH);

  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
\
  Serial.println("Connecting to server...");
  if (!client.connect(host, port)) {
    Serial.println("Connection to server failed");
    while(1); 
  }
  Serial.println("Connected to server");

  String pathData = client.readStringUntil('\n'); 
  String eventData = client.readStringUntil('\n'); 
  Serial.println("Received path data: " + pathData);
  Serial.println("Received event data: " + eventData);

  parseAndPopulateArray(pathData);
  parseAndPopulateArrayString(eventData);

  delay(5000);

}

// *********************************************************************

/* 
    Gets the current readings of the sensors and then updates the state 
    according to the new values. It then chooses the appropriate Line Following 
    algorithm accorrding to current state. 
*/

void loop() {
  updateCoords();
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
      right(ls*1.1, 0);
    }

  } else if (rv) { 

    els = RIGHT;

    while (!erv) {
      getValues();
      left(0, rs*1.1);
    }

  } else if (!elv && !erv) {

    moveForward();

  } else if (elv) {

    els = LEFT;

    if (!lv) {
      right(ls*1.1, 0);
    } else if (lv) {
      while (!elv) {
        getValues();
        right(ls*1.1, 0);
      }
    }

  } else if (erv) {

    els = RIGHT;

    if (!rv) {
      left(0, rs*1.1);
    } else if (rv) { 
      while (!erv) {
        getValues();
        left(0, rs*1.1);
      }
    }
  }
}

// *********************************************************************

/*
    Line following algorithm for following the roads of type-2. It uses the 
    Left, Middle, and Right sensors to navigate by sensing the line that 
    runs through the middle of the road.

    Abbreviations:
    LS: Left Sensor
    MS: Middle Sensor
    RS: Right Sensor

    -> If only the MS is detecting the line (LS = 0, MS = 1, RS = 0), move forward.
    -> If both the LS and MS are detecting the line (LS = 1, MS = 1, RS = 0), 
       turn slightly to the left
    -> If both LS and MS are detecting the line (LS = 1, MS = 0, RS = 0), 
       turn to the left more sharply.
    -> If both MS and RS is detecting the line (LS = 0, MS = 1, RS = 1), 
       turn slightly to the right.
    -> If only RS is detecting the line (LS = 0, MS = 0, RS = 1), 
       turn to the right more sharply.
    -> If all sensors (LS, MS, RS) are detecting the line, it indicates a node. 
       Execute the nodeRoutine() function to determine the next action.
    -> If no sensors are detecting the line (LS = 0, MS = 0, RS = 0), and the robot 
       is at the last node (nextNode == 0 and currNode == 1), stop the robot, signal 
       completion, and enter an infinite loop to cease further movement. 
       Otherwise, continue moving forward, assuming temporary loss of the line.
*/

void middleLineFollowing() {

  if (lv == 0 and mv == 1 and rv == 0) {
    moveForward();
  } else if (lv == 1 and mv == 0 and rv == 0) {
    left(ls*0.2, rs*0.95);
  } else if (lv == 1 and mv == 1 and rv == 0) {
    left(ls*0.5, rs*0.88);
  } else if (lv == 0 and mv == 0 and rv == 1) {
    right(ls*0.95, rs*0.2);
  } else if (lv == 0 and mv == 1 and rv == 1) {
    right(ls*0.88, rs*0.5);
  } else if (lv == 1 and mv == 1 and rv == 1) {
    nodeRoutine();
  } else if (lv == 0 and mv == 0 and rv == 0) {
      if (nextNode == 0 && currNode == 1) {
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
  checkNode1();
}

// *********************************************************************

/*
    Node decision-making algorithm to determine the direction of turn based 
    on the robot's current, previous, and next nodes. It uses a cross-product 
    function to decide on straight, left, right, or U-turn maneuvers.

    -> Increment the node iterator 'i' to move to the next node in the path array.

    -> Update 'prevNode', 'currNode', and 'nextNode' based on the positions in 
       the path array. 'prevNode' is the node just passed, 'currNode' is the node 
       where the robot currently is, and 'nextNode' is where the robot is headed.

    -> Calculate the turn direction using a cross-product function that takes 
       the coordinates of 'prevNode', 'currNode', and 'nextNode' to determine the 
       relative direction of the next turn.

    -> Check if a U-turn is required using the 'isUturn' function, which compares 
       'prevNode', 'currNode', and 'nextNode'. If a U-turn is needed, perform the 
       U-turn maneuver.

    -> If the cross-product result is 0, it indicates a straight path 
       or slight adjustments. The robot continues forward, making slight left or right 
       adjustments based on sensor readings until it aligns back on the path.

    -> If 'turnValue' is -1, it indicates a right turn is needed. The robot performs a 
       right turn until the middle sensor realigns with the path.

    -> If 'turnValue' is 1, it indicates a left turn is needed. The robot performs a 
       left turn until the middle sensor realigns with the path.


*/

void checkNode1() {
  i++;
  if (i == 0) {
    prevNode = 0;
    currNode = 0;
    nextNode = 1;
  } else {
    prevNode = path[i-1];
    currNode = path[i];
    nextNode = path[i+1];
  }

  int turnValue = crossProduct(prevNode, currNode, nextNode);

  if (isUturn(prevNode, currNode, nextNode)) {
    uTurn();
  } else if (turnValue == 0) {
    while (1) {
      getValues();
      if (lv && rv && mv) {
        moveForward();
      } else if (lv && !rv) {
        left(100, rs);
      } else if (!lv && rv) {
        right(ls, 110);
      } else if (!lv && !rv && mv) {
        break;
      }
    }
  } else if (turnValue == -1) {
    do {
      right(ls*0.9, 0);
    } while (!(!digitalRead(leftSensor) && digitalRead(middleSensor) && !digitalRead(rightSensor)));
  } else if (turnValue == 1) {
    do {
        left(0, rs*0.9);
      } while (!(!digitalRead(leftSensor) && digitalRead(middleSensor) && !digitalRead(rightSensor)));
  }
}

// *********************************************************************

/*
    The checkNode1() function determines the action to take at each node 
    based on the path. It updates the current, previous, and next node 
    indices, calculates the turn direction using a cross product, and decides 
    whether to perform a U-turn, continue straight, or make a left or right turn.

    -> Increment the node index 'i' to move to the next node in the path.

    -> Update the 'prevNode', 'currNode', and 'nextNode' variables based on 
       the current position in the path array. This step initializes the node 
       tracking at the start and updates it as the robot moves through the path.

    -> Calculate the 'turnValue' using the crossProduct function, which 
       determines the direction to turn based on the vector from the previous 
       node to the current node and the vector from the current node to 
       the next node.

    -> Check if a U-turn is necessary using the isUturn function. If a 
       U-turn is required, execute the uTurn() function.

    -> If 'turnValue' is 0, the path continues straight. The robot moves 
       forward until it aligns with the next path segment, adjusting its 
       direction based on sensor readings.

    -> If 'turnValue' is -1, a right turn is needed. Execute a right turn until 
       the robot is aligned with the next segment of the path (only the middle 
       sensor detects the line).

    -> If 'turnValue' is 1, a left turn is needed. Execute a left turn until the 
       robot is aligned with the next segment of the path (only the middle 
       sensor detects the line).
*/

int crossProduct(int pn, int cn, int nn) {

  int prev[2] = {nodeCoords[prevNode][0], nodeCoords[prevNode][1]};
  int curr[2] = {nodeCoords[currNode][0], nodeCoords[currNode][1]};
  int next[2] = {nodeCoords[nextNode][0], nodeCoords[nextNode][1]};

  int dx1 = curr[0] - prev[0];
  int dx2 = next[0] - curr[0];
  int dy1 = curr[1] - prev[1];
  int dy2 = next[1] - curr[1];

  if ((cn == 9 && nn == 11 && pn == 6) || (cn == 11 && nn == 9 && pn == 8)) {
    return 0;
  } else if (cn == 9 && nn == 11 && pn == 10) {
    return -1;
  } else if (cn == 11 && nn == 9 && pn == 10) {
    return 1;
  } else if (cn == 9 && nn == 10 && pn == 11) {
    return 1;
  } else if (cn == 11 && nn == 10 && pn == 9) {
    return -1;
  } else if (cn == 9 && nn == 6 && pn == 11) {
    return 0;
  } else if (cn == 11 && nn == 8 && pn == 9) {
    return 0;
  } else {
    return (dx1 * dy2) - (dy1 * dx2);
  }
}

/*
    If the previous node and next node are the same, then it returns true.
*/

bool isUturn(int pn, int cn, int nn) {
  if (nn == pn) {
    return true;
  } else {
    return false;
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
  analogWrite(lpwm, ls*0.88);
  analogWrite(rpwm, rs*0.88);

  Serial.println("move forward");
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rb, LOW);
  digitalWrite(rf, HIGH);
}

void moveBackward() {
  analogWrite(lpwm, ls*0.9);
  analogWrite(rpwm, rs*0.9);

  Serial.println("move backward");
  digitalWrite(lb, HIGH);
  digitalWrite(lf, LOW);
  digitalWrite(rb, HIGH);
  digitalWrite(rf, LOW);
}

void stop() {
  analogWrite(lpwm, ls*0.9);
  analogWrite(rpwm, rs*0.9);

  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  Serial.println("Stop");
}

void powerRight() {
  analogWrite(lpwm, 0.88*ls);
  analogWrite(rpwm, 0.91*rs);

  digitalWrite(lf, HIGH);
  digitalWrite(lb, LOW);
  digitalWrite(rb, HIGH); 
  digitalWrite(rf, LOW);
  Serial.println("Right");
}

void uTurn() {
  do {
    powerRight();
  } while (!(!digitalRead(leftSensor) && digitalRead(middleSensor) && !digitalRead(rightSensor)));
}

void getValues() {
  elv = digitalRead(extremeLeftSensor);
  lv = digitalRead(leftSensor);
  mv = digitalRead(middleSensor);
  rv = digitalRead(rightSensor);
  erv = digitalRead(extremeRightSensor);
}

// *********************************************************************

/*
    Converts the path received in string format into an array.
*/

void parseAndPopulateArray(String input) {
  int startIndex = input.indexOf('[') + 1;
  int endIndex = input.indexOf(']');
  String values = input.substring(startIndex, endIndex);

  int index = 0;
  while (values.length() > 0) {
    int commaIndex = values.indexOf(',');
    if (commaIndex == -1) {
      path[index] = values.toInt();
      break;
    }
    String numStr = values.substring(0, commaIndex);
    path[index] = numStr.toInt();
    values = values.substring(commaIndex + 2); // Skip comma and space
    index++;
  }

  // Print the array for verification
  for (int i = 0; i <= index; i++) {
    Serial.print("array[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(path[i]);
  }
}

/*
    Converts the events path received in string format into an array.
*/

void parseAndPopulateArrayString(String input) {
  int eventsCount = 0;
  for (int i = 0; i < input.length(); i++) {
    if (isAlpha(input.charAt(i)) && isUpperCase(input.charAt(i))) {
      // Check if the character is an uppercase letter
      events[eventsCount] = input.charAt(i);
      eventsCount++;
    }
  }

  events[eventsCount] = '\0';
  for (int i=0; i < strlen(events); i++) {
    Serial.println(events[i]);
  }
}

/*

    Continously received the updated location of the Vanguard. 
    It then compares with specified event location threshold.
    If it is within a bounding box, it will stop. 
    
*/

void updateCoords() {
  int32_t xpos, ypos;
  if (client.available() >= 8) {
    
    client.read((uint8_t *)&xpos, sizeof(xpos));
    client.read((uint8_t *)&ypos, sizeof(ypos));
  }

  if (xpos >= 285 && xpos <= 316 && ypos <= 832 && ypos >= 711 && events[j] == 'A') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 713 && xpos <= 777 && ypos <= 640 && ypos >= 508 && events[j] == 'B') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 720 && xpos <= 767 && ypos >= 338 && ypos <= 446 && events[j] == 'C') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 253 && xpos <= 315 && ypos >= 330 && ypos <= 453 && events[j] == 'D') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 284 && xpos <= 330 && ypos >= 51 && ypos <= 152 && events[j] == 'E') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else {
    isStopped = false;
  }
}