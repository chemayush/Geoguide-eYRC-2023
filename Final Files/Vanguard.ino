/* 
File: Vanguard.ino
Team ID: GG-1949
Author(s): Ayush Karapagale, Akshit Gangwar, Sparsh Gautam, Prakhar Shukla
Theme: GeoGuide
Functions: setup(), loop(), extremeLineFollowing(), middleLineFollowing(), switchFollowing(), updateState(), nodeRoutine(), checkNode1(), getValues(), crossProduct(), updateCoords(), isUturn(), parseAndPopulateArrayString(), parseAndPopulateArray(), right(), left(), moveForward(), moveBackward(), uTurn(), powerRight(), stop(), nonBlockingDelay()
Global Variables: ssid, password, host, port, client, extremeLeftSensor, leftSensor, middleSensor, rightSensor, extremeRightSensor, previousMillis, currentMillis, motor pins, sensor readings, motor speeds, LineFollowingState, node, nodeCoords, prevNode, currNode, nextNode, coordinatesString, pathString, eventString, path, events, i, j, isStopped, xpos, ypos
*/

#include <WiFi.h>

// WiFi Credentials and IP address
const char* ssid = "ayush";
const char* password = "raspberrypi";
const char* host = "192.168.220.211";
const uint16_t port = 8080;

WiFiClient client;

// IR Sensor pins
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
const int rb = 18;            //rb: Left Backward
const int rf = 19;            //lb: Left Forward
const int rpwm = 5;           //rpwm: Left PWM

const int lb = 22;            //lb: Right Backward
const int lf = 21;            //lf: Right Forward
const int lpwm = 23;          //lpwm: Right PWM

const int led = 2;            //led: Green LED 
const int buzzer = 15;        //buzzer: Buzzer

int elv, lv, mv, rv, erv;     // Sensor Readings

int ls = 125;                 //ls: Left Motor Speed
int rs = 120;                 //rs: Right Motor Speed (It is lesser than rs because right motor was turning a bit faster than left)

enum LineFollowingState {

  MIDDLE_LINE,
  EXTREME_LINE,

  RIGHT,
  LEFT

};

LineFollowingState state = MIDDLE_LINE;       //state: State of the line following depending on the type of road.
LineFollowingState els = LEFT;                //els: State to determine if it is on right side of the road or left side.

int node = 0;                                 //node: Keeps the count of nodes.
int nodeCoords[12][2] = { {0, 0}, 
                               {0, 1}, {1, 1},
                               {0, 2}, {1, 2}, {2, 2},
                               {0, 3}, {1, 3}, {2, 3},
                               {0, 4}, {1, 4}, {2, 4},
                             };                           // coordinates assigned to each node in the order: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11   

int prevNode = 0;                           //prevNode: Previously visited node.
int currNode = 0;                           //currNode: Currently visited node.
int nextNode = 1;                           //nextNode: Node to be visited next.

String coordinatesString = "";              //coordinatesString: String containing the coordintes of vanguard, as received through socket connection.
String pathString = "";                     //pathString: String containing the entire path to be followed, as received through socket connection.
String eventString = "";                    //eventString: String containing all the events to be visited, as received through socket connection.

int path[100] = {};                         //path[]: The path received as string is converted into an array of integral nodes and stored in this variable.
char events[10] = {};                       //events[]: The eventa received as string are converted to an array of characters and stored in this variable

int i = 0;                                  //i: node iterator
int j = 0;                                  //j: events iterator
bool isStopped = false;                     //isStopped: Vanguard is stopped at an event or not
float xpos, ypos;                           //xpos, ypos: Position of the Vanguard

// ******************************************************* All the functions used ********************************************************

void extremeLineFollowing();                      // Line following for roads of type 1.
void middleLineFollowing();                       // Line following for roads of type-2.
void switchFollowing();                           // Switches between middleLineFollowing() and extremeLineFollowing().
void updateState();                               // updates the state of line following, i.e. the type of road being followed right now.
void nodeRoutine();                               // Function executed when a node is detected.
void checkNode1();                                // Checks which node it is on right now.
int crossProduct(int pn, int cn, int nn);         // Calculates the cross product at each node to determine next direction.
void updateCoords();                              // Updates the current coordinates of the Vanguard as received through socket connection.
void getValues();                                 // Updates the values of ir sensors.
bool isUturn(int pn, int cn, int nn);             // Checks if there is a uTurn to be taken
void parseAndPopulateArrayString(String input);   // Converts the events string to array of characters.
void parseAndPopulateArray(String input);         // Converts the path string to an array of integral nodes.

// NOTE: The speeds in the usage of these functions have been optimally chosen based on arena and our Vanguard Robot's form factor.

void right(int l, int r);         // Vanguard moves right.
void left(int l, int r);          // Vanguard moves leftt.
void moveForward();               // Vanguard moves forward.
void moveBackward();              // Vanguard moves backwards.
void stop();                      // Vanguard stops.
void uTurn();                     // Vanguard makes a U Turn.
void nonBlockingDelay(int intv);  // A non blocking delay. A replacement for traditional delay() function.

// ***************************************************************************************************************************************

/* 
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Initializes the robot's sensors, motors, establishes WiFi connection, connects to socket and receives the path and events.
 * Example Call: Automatically called by Arduino framework.
 */

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

/* 
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic: Gets the current readings of the sensors and then updates the state 
       according to the new values. It then chooses the appropriate Line Following 
       algorithm accorrding to current state.
 * Example Call: Automatically called by Arduino framework in a loop.
 */

void loop() {
  updateCoords();
  getValues();          
  updateState();
  switchFollowing();
}

/* 
 * Function Name: updateState
 * Input: None
 * Output: None
 * Logic: updates the state of line following based on current sensor readings.
 * Example Call: updateState()
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

/* 
 * Function Name: switchFollowing
 * Input: None
 * Output: None
 * Logic: Switches between different line following states.
 * Example Call: switchFollowing()
 */

void switchFollowing() {

  if (state == MIDDLE_LINE) {
    middleLineFollowing();
  }
  
  if (state == EXTREME_LINE) {
    extremeLineFollowing();
  }

}

/*
 * Function Name: extremeLineFollowing
 * Input: None
 * Output: None
 * Logic:  Line following algorithm for following the roads of type-1. It uses the
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

 * Example Call: extremeLineFollowing(). Called within the loop() function when the robot is in the EXTREME_LINE state.
 */

void extremeLineFollowing() {

  getValues();
  
  if (lv) {

    els = LEFT;

    while (!elv) {
      getValues();
      right(ls*1.17, 0);
    }

  } else if (rv) { 
    els = RIGHT;

    while (!erv) {
      getValues();
      left(0, rs*1.17);
    }

  } else if (!elv && !erv) {

    moveForward();

  } else if (elv) {

    els = LEFT;

    if (!lv) {
      right(ls*1.17, 0);
    } else if (lv) {
      while (!elv) {
        getValues();
        right(ls*1.17, 0);
      }
    }

  } else if (erv) {

    els = RIGHT;

    if (!rv) {
      left(0, rs*1.17);
    } else if (rv) { 
      while (!erv) {
        getValues();
        left(0, rs*1.17);
      }
    }
  }
}

/* 
 * Function Name: middleLineFollowing
 * Input: None
 * Output: None
 * Logic:  Line following algorithm for following the roads of type-2. It uses the 
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

 * Example Call: Called within the loop() function when the robot is in the MIDDLE_LINE state.

 */

void middleLineFollowing() {

  if (lv == 0 and mv == 1 and rv == 0) {
    moveForward();
  } else if (lv == 1 and mv == 0 and rv == 0) {
    left(ls*0.28, rs*0.95);
  } else if (lv == 1 and mv == 1 and rv == 0) {
    left(ls*0.5, rs*0.88);
  } else if (lv == 0 and mv == 0 and rv == 1) {
    right(ls*0.95, rs*0.28);
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

/* 
 * Function Name: nodeRoutine
 * Input: None
 * Output: None
 * Logic: This function is executed when a node is encountered.
    
    -> 'node' variable is incremented.
    -> it executes the checkNode() function

 * Example Call: nodeRoutine(). Called when the robot detects a node.
 */

void nodeRoutine() {
  node++;
  checkNode1();
}

/*
 * Function Name: checkNode1
 * Input: None:
 * Output: None
 * Logic:  Node decision-making algorithm to determine the direction of turn based 
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

 * Example Call: checkNode1(). Called within nodeRoutine function.
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
        left(45, rs);
      } else if (!lv && rv) {
        right(ls, 50);
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

/*
 * Function Name: crossProduct
 * Input: pn - Previous node identifier,
          cn - Current node identifier (not used, included for potential future use),
          nn - Next node identifier
 * Output: return -1, 0, 1 based on the the calculation.
 * Logic: This function calculates the cross product of the vectors formed by the points at previous, current, and next nodes to determine the relative direction of the path at the current node. It incorporates special cases for handling specific node sequences that represent predefined maneuvers or adjustments in the path.
 * Example Call: int result = crossProduct(prevNode, currNode, nextNode);
 */

int crossProduct(int pn, int cn, int nn) {

  int prev[2] = {nodeCoords[prevNode][0], nodeCoords[prevNode][1]};
  int curr[2] = {nodeCoords[currNode][0], nodeCoords[currNode][1]};
  int next[2] = {nodeCoords[nextNode][0], nodeCoords[nextNode][1]};

  int dx1 = curr[0] - prev[0];
  int dx2 = next[0] - curr[0];
  int dy1 = curr[1] - prev[1];
  int dy2 = next[1] - curr[1];

  if (cn == 9 || cn == 11 || (cn == 5 && nn == 2) || (pn == 5 && cn == 2)) {
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
    } else if (cn == 2 && nn == 1 && pn == 5) {
      return 0;
    } else if (cn == 5 && nn == 2 && pn == 4) {
      return -1;
    } else if (cn == 5 && nn == 2 && pn == 8) {
      return 0;
    } 
  } else {  
    return (dx1 * dy2) - (dy1 * dx2);
  }
}

 /*
  * Function Name: isUturn
  * Input: pn - Previous node identifier,
           cn - Current node identifier (not used, included for potential future use),
           nn - Next node identifier
    Output: Returns true if the next node is the same as the previous node, indicating a U-turn. Otherwise, returns false.
  * Logic: This function checks if the vanguard is making a U-turn based on its previous and next positions.
           A U-turn is identified if the previous and next positions are the same.
  * Example Call: bool result = isUturn(1, 2, 1);
  */

bool isUturn(int pn, int cn, int nn) {
  if (nn == pn) {
    return true;
  } else {
    return false;
  }
}

 /*
  * Function Name: nonBlockingDelay
  * Input: intv - Interval for the non-blocking delay in milliseconds.
  * Output: None
  * Description: This function provides a non-blocking delay mechanism. Unlike the traditional delay() function,
                which halts the CPU, making it unresponsive, this function allows for other operations to be
                performed while waiting. It relies on the millis() function to keep track of time elapsed.
                This approach ensures that the system remains responsive, especially when sensor inputs are critical.
                The function assumes that 'previousMillis' and 'currentMillis' are declared as global variables.
  * Logic: The function records the start time, then continuously checks if the specified interval has elapsed
          by comparing the current time with the start time. During this period, it calls the stop() function,
          allowing for other operations or checks to be performed, thus making it non-blocking.
  * Example Call: nonBlockingDelay(1000); // Non-blocking delay of 1000 milliseconds
  */

void nonBlockingDelay(int intv) {

  previousMillis = millis();
  while (currentMillis - previousMillis < intv) {
    stop();
    currentMillis = millis();
  }

}

/*
* Function Name: left
* Input: int l, int r
* Output: None
* Logic: Takes a left by lowering right speed.
* Example Call: left(0, 200);
*/

void left(int l, int r) {
  analogWrite(lpwm, l);
  analogWrite(rpwm, r);

  digitalWrite(rb, LOW); 
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rf, HIGH);
  Serial.println("leftt");
}

/*
* Function Name: right
* Input: int l, int r
* Output: None
* Logic: Takes a right by lowering left speed.
* Example Call: right(200, 0);
*/

void right(int l, int r) {
  analogWrite(lpwm, l);
  analogWrite(rpwm, r);

  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rf, HIGH); 
  digitalWrite(rb, LOW);
  Serial.println("right");
}

/*
* Function Name: moveForward
* Input: None
* Output: None
* Logic: moves straight by keeping the right and left speed similar.
* Example Call: moveForward();
*/

void moveForward() {
  analogWrite(lpwm, ls*0.75);
  analogWrite(rpwm, rs*0.75);

  Serial.println("move forward");
  digitalWrite(lb, LOW);
  digitalWrite(lf, HIGH);
  digitalWrite(rb, LOW);
  digitalWrite(rf, HIGH);
}

/*
* Function Name: stop
* Input: None
* Output: None
* Logic: stops by toggling the motor values to LOW.
* Example Call: stop();
*/

void stop() {
  analogWrite(lpwm, ls*0.9);
  analogWrite(rpwm, rs*0.9);

  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  Serial.println("Stop");
}

/*
* Function Name: powerRight
* Input: None
* Output: None
* Logic: Takes a right by reversing right motor.
* Example Call: powerRight(); (called in u turn function)
*/

void powerRight() {
  analogWrite(lpwm, 0.88*ls);
  analogWrite(rpwm, 0.91*rs);

  digitalWrite(lf, HIGH);
  digitalWrite(lb, LOW);
  digitalWrite(rb, HIGH); 
  digitalWrite(rf, LOW);
  Serial.println("Right");
}

/*
* Function Name: uTurn
* Input: None
* Output: None
* Logic: Takes a uTurn by calling powerRight() function until a middle line is found.
* Example Call: left();
*/

void uTurn() {
  do {
    powerRight();
  } while (!(!digitalRead(leftSensor) && digitalRead(middleSensor) && !digitalRead(rightSensor)));
}

/*
* Function Name: getValues
* Input: None
* Output: None
* Logic: updates sensor values
* Example Call: getValues();
*/

void getValues() {
  elv = digitalRead(extremeLeftSensor);
  lv = digitalRead(leftSensor);
  mv = digitalRead(middleSensor);
  rv = digitalRead(rightSensor);
  erv = digitalRead(extremeRightSensor);
}

/*
* Function Name: parseAndPopulateArray
* Input: String input (received through socket)
* Output: None
* Logic: splits input into a integer array.
* Example Call: parseAndPopulateArray(String input);
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

  for (int i = 0; i <= index; i++) {
    Serial.print("array[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(path[i]);
  }
}

/*
* Function Name: parseAndPopulateArrayString
* Input: String input (received through socket)
* Output: None
* Logic: splits inputt into a character array.
* Example Call: parseAndPopulateArray(String input);
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
* Function Name: updateCoords
* Input: SNone
* Output: None
* Logic: Continously received the updated location of the Vanguard. 
         It then compares with specified event location threshold.
         If it is within a bounding box, it will stop. 
* Example Call: updateCoords();
*/

void updateCoords() {
  int32_t xpos, ypos;
  if (client.available() >= 8) {
    
    client.read((uint8_t *)&xpos, sizeof(xpos));
    client.read((uint8_t *)&ypos, sizeof(ypos));
  }

  if (xpos >= 245 && xpos <= 286 && ypos <= 832 && ypos >= 711 && events[j] == 'A') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 703 && xpos <= 767 && ypos <= 640 && ypos >= 508 && events[j] == 'B') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 725 && xpos <= 762 && ypos >= 338 && ypos <= 446 && events[j] == 'C') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 263 && xpos <= 305 && ypos >= 330 && ypos <= 453 && events[j] == 'D') {
    if (!isStopped) {
      stop();
      digitalWrite(buzzer, LOW);
      nonBlockingDelay(1000);
      digitalWrite(buzzer, HIGH);
      moveForward();
      j++;
      isStopped = true;
    }
  } else if (xpos >= 270 && xpos <= 293 && ypos >= 51 && ypos <= 152 && events[j] == 'E') {
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