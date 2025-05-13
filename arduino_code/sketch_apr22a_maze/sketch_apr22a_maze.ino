#include "LIDARLite_v4LED.h"
#include <Wire.h>
#include <LiquidCrystal.h>
#include "Adafruit_TCS34725.h"
#include <EEPROM.h>

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10
#define button_pin 18

#define redpin 3
#define greenpin 5
#define bluepin 6

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];

// Constants
LIDARLite_v4LED myLIDAR;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
const int compassAddress = 0x60;
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
const int pulsesPerCm = 80;
const int max_speed = 255;
const int ENCA = 2;
const int ENCB = 3;
const int VRx = A9;
const int VRy = A10;

// LCD Initialization
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

bool turnRightNext = true;
bool findNorthBoolean = true;
bool findSouthBoolean = true;

bool greenPassed = false;
bool bluePassed = false;
bool sequenceStarted = false;

// Global variables
volatile int pulse_count_A = 0;
volatile int pulse_count_B = 0;
int distance_to_drive = 0;  // Distance read from Serial
int pwm_R = 0; // speed of both wheels
int pwm_L = 0;
int target_heading = -1;
bool rightForward = true;
int direction_right = 1;
int counter = 0; // 0 - esp mode, 1 - joystick to get Red, 2 get Green, 3 get Blue
int offset = 0;
int degrees = 0; //current degrees
String direction = ""; // "N", "E", "NW" etc.
int analogX;
int analogY;
int newDistance;
int targetDistance;
String lidarData;
float encoderData;
String modeString = "Initial";
unsigned long previousMillis = 0;
int sendInterval = 10000;
int stepIndex = 0;
String last_command = "";
int totalPulses = 0;
int turningAngle = -1;

int tapeRed[] = {0, 0, 0};
int tapeGreen[] = {0, 0, 0};
int tapeBlue[] = {0, 0, 0};

int currentColors[] = {0, 0, 0};

float speed_for_L = 0.45;
float speed_for_R = 0.49;

void setup() {
  Wire.begin();
  lcd.begin(20, 4);
  Serial.begin(9600);
  Serial.println("Qwiic LIDARLite_v4 examples");
  Serial2.begin(9600);

  // Attach interrupts for encoders and mode toggle
  attachInterrupt(digitalPinToInterrupt(ENCA), pin_ISR_ENCA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCB), pin_ISR_ENCB, FALLING);
  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), pin_ISR, FALLING);
  getOffset();

  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }

  Serial.println("LIDAR acknowledged!");
  attachInterrupt(digitalPinToInterrupt(ENCA), pin_ISR_ENCA, FALLING);

  if (tcs.begin()) {
      //Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1); // halt!
    }

  #if defined(ARDUINO_ARCH_ESP32)
    ledcAttach(redpin, 12000, 8);
    ledcAttach(greenpin, 12000, 8);
    ledcAttach(bluepin, 12000, 8);
  #else
    pinMode(redpin, OUTPUT);
    pinMode(greenpin, OUTPUT);
    pinMode(bluepin, OUTPUT);
  #endif

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
}

// get offset at the setup
void getOffset() {
  offset = -get_current_heading();
}

// Fetch the current heading from the compass
int get_current_heading() {
  Wire.beginTransmission(compassAddress);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(compassAddress, 1, true);

  if (Wire.available() >= 1) {
    int bearing = Wire.read();
    degrees = (int)bearing / 255.0 * 360.0; // Convert to degrees
  }

  degrees += offset + 360;
  degrees = degrees % 360;
  return degrees;
}

// Update the LCD display only when necessary
void updateLCD(String line1, String line2) {
  static String lastLine1 = "";
  static String lastLine2 = "";
  if (line1!= lastLine1 || line2 != lastLine2) {
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
    lastLine1 = line1;
    lastLine2 = line2;
  }
}

float average(float samples) {
  float distanceSum = 0;
  for (int i = 0; i < samples; i++) {
    newDistance = myLIDAR.getDistance();
    distanceSum += newDistance * 1.0;
    delay(10);
  }
  return distanceSum / samples;
}

// Get direction based on degrees
String getDirection(int degrees) {
  if (degrees >= 22.5 && degrees < 67.5) return "NE";
  if (degrees >= 67.5 && degrees < 112.5) return "E";
  if (degrees >= 112.5 && degrees < 157.5) return "SE";
  if (degrees >= 157.5 && degrees < 202.5) return "S";
  if (degrees >= 202.5 && degrees < 247.5) return "SW";
  if (degrees >= 247.5 && degrees < 292.5) return "W";
  if (degrees >= 292.5 && degrees < 337.5) return "NW";
  return "N"; // degrees >= 337.5 || degrees < 22.5
}

// pulses and distance, from beginning to stop of the last action
void updateDistance() {
  lcd.setCursor(0, 2);
  lcd.print((String) pulse_count_A + " " + (String) (pulse_count_A / (pulsesPerCm * 1.0)));
  lcd.setCursor(10, 2);
  lcd.print((String) pulse_count_B + " " + (String) (pulse_count_B / (pulsesPerCm * 1.0)));
}

// Handle commands from the ESP module
void esp_page() {
  if (Serial2.available() > 0 | target_heading == -1) {
    String message = Serial2.readStringUntil('\n');
    if (message.length() != 0) {
     Serial.println(message);

      int pos_dist = message.indexOf("Move:"); // we figure out what kind of message was sent
      int pos_degree = message.indexOf("Turn:");

      if (pos_degree > -1) { //it's a turn for a defined degree
        pos_degree = message.indexOf(":");
         Serial.println("hiii");
         last_command = "Turn";

        if (pos_degree > -1) {
          String stat = message.substring(pos_degree + 1);
          if (stat.toInt() == 360) { // turn for 360
            lcd.setCursor(0, 3);
            lcd.print("                 ");
            lcd.setCursor(0, 3);
            lcd.print("Turn: 360");
            turn360(1);
          } else if (stat.toInt() == -360) {
            lcd.setCursor(0, 3);
            lcd.print("                 ");
            lcd.setCursor(0, 3);
            lcd.print("Turn: -360");
            turn360(-1);
          }
          target_heading = stat.toInt() % 360; // turn for another degree
          int direction_right = target_heading > 0 ? 1 : -1;

          degrees = get_current_heading();
          lcd.setCursor(0, 3);
          lcd.print("                 ");
          lcd.setCursor(0, 3);
          lcd.print("Turn: " + (String) target_heading);
          if (degrees + target_heading > 0) {
            turn((degrees + target_heading) % 360, direction_right);
          } else {
            turn((degrees + (360 - (abs(target_heading)))), direction_right);
          }
        }
      } else if (pos_dist > -1) { // it's driving for some distance
        float lidarDist = average(5);

        distance_to_drive = message.substring(pos_dist + 5).toInt();
        lcd.setCursor(0, 3);
        lcd.print("                 ");
        lcd.setCursor(0, 3);
        lcd.print("Drive: " + (String) distance_to_drive);
        int rotateCount = 0;
        target_heading = 90;
        degrees = get_current_heading();
        while(lidarDist <= 10 && rotateCount < 4 && distance_to_drive > 0){

          if (degrees + target_heading > 0) {

            turn((degrees + target_heading) % 360, 1);
            Serial2.println("LIDAR:" + (String) average(1));  // Send data to Serial Monitor
            Serial2.println("Degree:" + String(degrees));
            get_color();
          }
          lidarDist = average(5);
          rotateCount += 1;
        }
        if (distance_to_drive > 0 && rotateCount < 4) {
            driveForwards(distance_to_drive);
          } else if (distance_to_drive < 0 && rotateCount < 4) {
            driveBackwards(abs(distance_to_drive));
          }
      }
    }
  }
}

void get_color() {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  if (counter == 1) {
    tapeRed[0] = int(red);
    tapeRed[1] = int(green);
    tapeRed[2] = int(blue);

  } else if (counter == 2) {
    tapeGreen[0] = int(red);
    tapeGreen[1] = int(green);
    tapeGreen[2] = int(blue);
  } else if (counter == 3) {
    tapeBlue[0] = int(red);
    tapeBlue[1] = int(green);
    tapeBlue[2] = int(blue);
  }

  Serial2.println("R: " + (String)(int)red + " " + (String)(int)green + " " + (String)(int)blue);
  currentColors[0] = int(red);
  currentColors[1] = int(green);
  currentColors[2] = int(blue);

  Serial.println("R: " + (String)(int)red + " " + (String)(int)green + " " + (String)(int)blue);
  Serial.println("Tape red: " + (String)tapeRed[0] + " " + (String)tapeRed[1] + " " + (String)tapeRed[2]);
  Serial.println("Tape green: " + (String)tapeGreen[0] + " " + (String)tapeGreen[1] + " " + (String)tapeGreen[2]);
  Serial.println("Tape blue: " + (String)tapeBlue[0] + " " + (String)tapeBlue[1] + " " + (String)tapeBlue[2]);

  #if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(1, gammatable[(int)red]);
    ledcWrite(2, gammatable[(int)green]);
    ledcWrite(3, gammatable[(int)blue]);
  #else
    analogWrite(redpin, gammatable[(int)red]);
    analogWrite(greenpin, gammatable[(int)green]);
    analogWrite(bluepin, gammatable[(int)blue]);
  #endif
}

int lastMode = -1;

void loop() {
  esp_page();
  get_color();
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
  Serial2.println("LIDAR:" + (String) average(3));  // Send data to Serial Monitor
  Serial2.println("Degree:" + String(degrees));
  degrees = get_current_heading();
  updateLCD(modeString, "Direction: " + String(degrees) + ", " + getDirection(degrees));

  if (counter != lastMode) {
    lastMode = counter;

    if (counter == 1 || counter == 2 || counter == 3) {
      updateLCD("Place sensor", modeString);
      delay(1000);
      get_color();
      updateLCD("Color Set", modeString);
    }
  }

  delay(100);
}



// Function to drive forwards for a specific distance
void driveForwards(int distance) {
  if (distance == 20) {
    Serial.println("Starting encoder calibration...");

    float averageInitialDistance = myLIDAR.getDistance();
    float stopDistance = averageInitialDistance - 20;
    Serial.println("LIDAR:" + (String) (myLIDAR.getDistance()));
    Serial.println("stopDistance:" + (String) stopDistance);


    if (stopDistance < averageInitialDistance - 0.03) {
      pulse_count_A = 0;
      while (stopDistance < averageInitialDistance - 0.03) {
        averageInitialDistance = myLIDAR.getDistance();
        digitalWrite(Motor_R_dir_pin, Motor_forward);
        digitalWrite(Motor_L_dir_pin, Motor_forward);
        pwm_L = max_speed * 0.47;
        pwm_R = max_speed * 0.51;
        analogWrite(Motor_L_pwm_pin, pwm_L);
        analogWrite(Motor_R_pwm_pin, pwm_R);
        Serial.println("updatedLidar:" + (String) (myLIDAR.getDistance()));

      }
    } else if (stopDistance > averageInitialDistance + 0.03) {
      pulse_count_A = 0;
      while (stopDistance > averageInitialDistance + 0.03) {
        averageInitialDistance = myLIDAR.getDistance();
        digitalWrite(Motor_R_dir_pin, Motor_return);
        digitalWrite(Motor_L_dir_pin, Motor_return);
        pwm_L = max_speed * 0.3;
        pwm_R = max_speed * 0.3;
        analogWrite(Motor_L_pwm_pin, pwm_L);
        analogWrite(Motor_R_pwm_pin, pwm_R);
      }
    }

    Serial.println("Stopping now!");
    stopMotor();
    delay(1000);

    totalPulses = pulse_count_A;
    Serial.println("Total Pulses: " + String(totalPulses));

    float encoderData = 200.0 / totalPulses ;
    Serial.println("Encoder Data: " + String(encoderData));

    EEPROM.put(0, encoderData);
    Serial2.println("Encoder:" + String(encoderData));
  }

  else if (distance == 500) {
    /// tapeRed 114 57 82
    /// tapeGreen 43 91 115
    /// tapeBlue 44 68 137
    bool move = true;
    findNorth();
    delay(500);

    get_color(); // Always get fresh color data at the start of each loop

    float lidarDistance = average(3);
    Serial.println("Update: " + String(lidarDistance));

    distanceWall(66);
    delay(500);
    findNorth();
    delay(500);

    lidarDistance = average(3);

    Serial.println(">>> About to call turnToDegree");
    turnToDegree(90, -1);  // left

    delay(500);

    while(move){
      get_color(); // Always get fresh color data at the start of each loop
      delay(50);
      Serial.println("Start to detect color:");
      Serial.print("Color Read: R=");
      Serial.print(currentColors[0]);
      Serial.print(" G=");
      Serial.print(currentColors[1]);
      Serial.print(" B=");
      Serial.println(currentColors[2]);

      if (lidarDistance < 12) {

        Serial.println("Wall, turning away");
        stopMotor();
        delay(500);
        driveBackwards(8);
        delay(500);

        turnToDegree(90, -1);

        delay(500);
        //continue;
      }

      if (bluePassed && lidarDistance < 12) {

        Serial.println("Blue sequence triggered: Wall detected after blue");

        stopMotor();
        delay(500);

        turnToDegree(90, -1);
        delay(500);

      }

      if (greenPassed && !sequenceStarted && lidarDistance < 12) {
        sequenceStarted = true;
        Serial.println("Green sequence triggered: Wall detected after green");

        stopMotor();
        delay(500);

        turnToDegree(90, 1);
        delay(500);

        distanceWall(10);
        delay(500);

        findNorth();
        delay(500);

        distanceWall(10);
        delay(500);

        // Optional: Stop the robot or set move = false if done
        move = false;
        Serial.println("Car stopped");
      }


      // if red
      if ((currentColors[0] <= (tapeRed[0] + 10) && currentColors[0] >= (tapeRed[0] - 10)) &&
        (currentColors[1] <= (tapeRed[1] + 10) && currentColors[1] >= (tapeRed[1] - 10)) &&
        (currentColors[2] <= (tapeRed[2] + 10) && currentColors[2] >= (tapeRed[2] - 10))
        ) {
        Serial.println("Red detected: turning");
        stopMotor();
        delay(500);
        driveBackwards(8);
        delay(500);

        if (turnRightNext) {
          turnToDegree(70, 1);
        } else {
          turnToDegree(90, -1);
        }
        turnRightNext = !turnRightNext;

        delay(500);
        continue;
      }

      // if blue
      else if ((currentColors[0] <= (tapeBlue[0] + 10) && currentColors[0] >= (tapeBlue[0] - 10)) &&
        (currentColors[1] <= (tapeBlue[1] + 10) && currentColors[1] >= (tapeBlue[1] - 10)) &&
        (currentColors[2] <= (tapeBlue[2] + 10) && currentColors[2] >= (tapeBlue[2] - 10))
      ) {
        speed_for_L = 0.30;
        speed_for_R = 0.34;
        delay(500);
        Serial.println("Blue detected: 35 percent");
      }

      // if green
      else if ((currentColors[0] <= (tapeGreen[0] + 10) && currentColors[0] >= (tapeGreen[0] - 10)) &&
        (currentColors[1] <= (tapeGreen[1] + 10) && currentColors[1] >= (tapeGreen[1] - 10)) &&
        (currentColors[2] <= (tapeGreen[2] + 10) && currentColors[2] >= (tapeGreen[2] - 10))
      ) {
        speed_for_L = 0.73;
        speed_for_R = 0.76;
        greenPassed = true;
        Serial.println("Green detected: 75 percent");
      }

      // if no tape
      else {
        speed_for_L = 0.45;
        speed_for_R = 0.50;
        Serial.println("Normal speed");
      }

      digitalWrite(Motor_R_dir_pin, Motor_forward);
      digitalWrite(Motor_L_dir_pin, Motor_forward);
      pwm_L = max_speed * speed_for_L;
      pwm_R = max_speed * speed_for_R;
      analogWrite(Motor_L_pwm_pin, pwm_L);
      analogWrite(Motor_R_pwm_pin, pwm_R);
      lidarDistance = average(3);
      Serial2.println("LIDAR:" + (String) average(3));
      Serial2.println("Degree:" + String(degrees));
    }
  }

  delay(500);
}

// Find north by turning the robot
void findNorth() {
  int currentHeading = get_current_heading();

  if (currentHeading > 180) {
    turningAngle = 0;
    direction_right = 1;
  } else {
    turningAngle = 0;
    direction_right = -1;
  }

  turn(turningAngle, direction_right);
  Serial.println("North find.");
}

void findSouth() {
  int currentHeading = get_current_heading();
  int targetHeading = 180;

  int direction;
  int angleDifference;

  if (currentHeading > targetHeading) {
    angleDifference = currentHeading - targetHeading;
    direction = -1; // turn left
  } else {
    angleDifference = targetHeading - currentHeading;
    direction = 1; // turn right
  }


  if (angleDifference > 180) {
    angleDifference = 360 - angleDifference;
    direction *= -1;
  }

  turn(angleDifference, direction);

  Serial.println("South find.");
}


void distanceWall(int targetDistance) {
  while(true) {
    float currentDistance = average(3);
    Serial.println("Current: " + String(currentDistance));
    if (currentDistance > targetDistance + 5) {
      digitalWrite(Motor_R_dir_pin,Motor_forward);
      digitalWrite(Motor_L_dir_pin,Motor_forward);
      pwm_L = max_speed * 0.45;
      pwm_R = max_speed * 0.51;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R);
      Serial.println("Move forward");
    } else if (currentDistance < targetDistance - 5) {
      digitalWrite(Motor_R_dir_pin,Motor_return);
      digitalWrite(Motor_L_dir_pin,Motor_return);
      pwm_L = max_speed * 0.45;
      pwm_R = max_speed * 0.51;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R);
      Serial.println("Move backward");
    } else {
      stopMotor();
      Serial.println("At target distance");
      break;
    }
  }
  delay(300);
}

// Function to drive backwards for a specific distance
void driveBackwards(int distance) {
  pulse_count_A = 0;
  int targetPulses = distance * pulsesPerCm;
  digitalWrite(Motor_R_dir_pin, Motor_return);
  digitalWrite(Motor_L_dir_pin, Motor_return);
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  Serial.println("Starting driveBackwards");
  while (pulse_count_A <= targetPulses) {
    pwm_L = max_speed * 0.28;
    pwm_R = max_speed * 0.32;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
    //Serial.println("Pulse Count: " + String(pulse_count_A));
    //Serial.println("PWM L: " + String(pwm_L) + ", PWM R: " + String(pwm_R));
    updateDistance();
  }
  stopMotor();
  //Serial.println("driveBackwards completed");
}

void driveDistance(int distance) {
  pulse_count_A = 0;
  int targetPulses = distance * pulsesPerCm;
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  digitalWrite(Motor_L_dir_pin, Motor_forward);
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  Serial.println("Starting drive distances");
  while (pulse_count_A <= targetPulses) {
    pwm_L = max_speed * 0.45;
    pwm_R = max_speed * 0.48;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    updateDistance();
  }
  stopMotor();
}



// Turn the robot to a target heading
void turn(int target_heading, int direction) {
    degrees = get_current_heading();

    while (abs(target_heading - get_current_heading()) >= 6) { // the turn is not completely accurate, an error of 6 degrees or less is considered fine
      updateDistance();
      float pwm_L = max_speed * 0.45;
      float pwm_R = max_speed * 0.50;
      if (direction == -1) {
        digitalWrite(Motor_R_dir_pin, Motor_forward);
        digitalWrite(Motor_L_dir_pin, Motor_return);
      } else {
        digitalWrite(Motor_R_dir_pin, Motor_return);
        digitalWrite(Motor_L_dir_pin, Motor_forward);
      }
      analogWrite(Motor_L_pwm_pin, pwm_L);
      analogWrite(Motor_R_pwm_pin, pwm_R);
    }
    stopMotor();
}


void turnToDegree(int target_heading, int direction) {
  Serial.println("turnToDegree function called");
  float start_heading = get_current_heading();
  float current_heading = start_heading;

  while (true) {
    updateDistance();
    current_heading = get_current_heading();

    // Calculate turned angle with wrap-around logic
    float turned_angle;
    if (direction == 1) { // right turn (clockwise)
      turned_angle = fmod((current_heading - start_heading + 360), 360);
    } else { // left turn (counterclockwise)
      turned_angle = fmod((start_heading - current_heading + 360), 360);
    }

    if (turned_angle >= target_heading - 6) break; // allow margin of error

    // Set PWM values
    float pwm_L = max_speed * 0.50;
    float pwm_R = max_speed * 0.54;

    // Set motor directions based on turn direction
    if (direction == -1) { // left
      digitalWrite(Motor_R_dir_pin, Motor_forward);
      digitalWrite(Motor_L_dir_pin, Motor_return);
    } else { // right
      digitalWrite(Motor_R_dir_pin, Motor_return);
      digitalWrite(Motor_L_dir_pin, Motor_forward);
    }

    // Apply PWM to motors
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
  }

  stopMotor();
  Serial.println("Turning 90 completed");
  delay(100);
}

// Function to make the robot have a full 360-degree rotate.
void turn360(int direction) {
  int base = get_current_heading();
  int target_heading = get_current_heading() + 360;
  float pwm_L = max_speed * 0.4;
  float pwm_R = max_speed * 0.4;
  while (abs(target_heading - get_current_heading()) >= 6) {
    if (direction == -1) {
        digitalWrite(Motor_R_dir_pin, Motor_forward);
        digitalWrite(Motor_L_dir_pin, Motor_return);
      } else {
        digitalWrite(Motor_R_dir_pin, Motor_return);
        digitalWrite(Motor_L_dir_pin, Motor_forward);
      }

      analogWrite(Motor_L_pwm_pin, pwm_L);
      analogWrite(Motor_R_pwm_pin, pwm_R);

      if (base != target_heading) {
        delay(500);
        target_heading = base;
    }
  }
  stopMotor();
}

// Handle joystick input
void joystick() {
  analogX = analogRead(VRx);
  analogY = analogRead(VRy);

  lcd.setCursor(0, 3);
  lcd.print("x: " + (String) analogX + " y: " + (String) analogY + "  ");

  // Map joystick input to motor control
  if (900 > analogX & analogX > 600) {
    updateDistance();
    digitalWrite(Motor_R_dir_pin,Motor_forward);
    digitalWrite(Motor_L_dir_pin,Motor_forward);
    pwm_L = max_speed * 0.5;
    pwm_R = max_speed * 0.5;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R);
  } else if (analogX >= 900) {
    updateDistance();
    digitalWrite(Motor_R_dir_pin,Motor_forward);
    digitalWrite(Motor_L_dir_pin,Motor_forward);
    pwm_L = max_speed;
    pwm_R = max_speed;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R);
  } else if (200 < analogX & analogX < 400) {
    updateDistance();
    digitalWrite(Motor_R_dir_pin,Motor_return);
    digitalWrite(Motor_L_dir_pin,Motor_return);
    pwm_L = max_speed * 0.5;
    pwm_R = max_speed * 0.5;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R);
  } else if (analogX <= 200) {
    updateDistance();
    digitalWrite(Motor_R_dir_pin,Motor_return);
    digitalWrite(Motor_L_dir_pin,Motor_return);
    pwm_L = max_speed;
    pwm_R = max_speed;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R);
  } else {
    if (analogY > 514) {
      updateDistance();
      digitalWrite(Motor_R_dir_pin,Motor_return);
      digitalWrite(Motor_L_dir_pin,Motor_forward);
      pwm_L = 100;
      pwm_R = 100;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R);
    } else if (analogY < 485) {
      updateDistance();
      digitalWrite(Motor_R_dir_pin,Motor_forward);
      digitalWrite(Motor_L_dir_pin,Motor_return);
      pwm_L = 100;
      pwm_R = 100;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R);
    } else {
      stopMotor();
    }
  }
}

// Function to stop the motor
void stopMotor() {
  pwm_L = 0;
  pwm_R = 0;
  analogWrite(Motor_L_pwm_pin, pwm_L);
  analogWrite(Motor_R_pwm_pin, pwm_R);
}

// ISR(Interrupt service routine) for encoder A
void pin_ISR_ENCA() {
  pulse_count_A++;
}

// ISR for encoder B
void pin_ISR_ENCB() {
  pulse_count_B++;
}

// ISR for toggling joystick or webpage mode
void pin_ISR() {
  delay(1000);
  counter += 1;
  counter %= 4;

  if (counter == 0) {
    modeString = "Normal ESP";
  } else if (counter == 1) {
    modeString = "Red       ";
  } else if (counter == 2) {
    modeString = "Green     ";
  } else if (counter == 3) {
    modeString = "Blue      ";
  }

  updateLCD("Color calibration", modeString);
}
