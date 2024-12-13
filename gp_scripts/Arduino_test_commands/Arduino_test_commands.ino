#include <SoftwareSerial.h>
#include <TinyGPS++.h>
// Pin assignments for motor driver (update based on your setup)
#define ENA 3 // PWM pin for motor A
#define ENB 6 // PWM pin for motor B
#define IN1 4  // Direction pin for motor A
#define IN2 2  // Direction pin for motor A
#define IN3 7  // Direction pin for motor B
#define IN4 5  // Direction pin for motor B
SoftwareSerial gpsSerial(9,8);

TinyGPSPlus gps;
long lastGpsUpdate = 0;   // Last GPS data update time
long gpsUpdateInterval = 500; // Update GPS data every 0.5 seconds

// Function to set motor speeds
void setMotorSpeeds(int speedLeft, int speedRight) {
  if(speedLeft==0 && speedRight==0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 0);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 0);
    return;
  }
  // Ensure speed values are within 0-100
  speedLeft = constrain(speedLeft, -100, 100);
  speedRight = constrain(speedRight, -100, 100);

  // Map speed values to PWM range (0-255)
  int pwmLeft = map(abs(speedLeft), 0, 100, 0, 255);
  int pwmRight = map(abs(speedRight), 0, 100, 0, 255);

  // Set left motor direction and speed
  if (speedLeft >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmLeft);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwmLeft);
  }

  // Set right motor direction and speed
  if (speedRight >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmRight);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwmRight);
  }
}

// Function to handle commands
void handleCommand(String command) {
  if(command=="getGpsData")
  {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float altitude = gps.altitude.meters();

    // Format the GPS data as specified
    String gpsData = String(latitude, 6) + "&" + String(longitude, 6) +
                     "&" + String(altitude, 2);
      
    // Send the formatted GPS data to the connected device
    Serial.println(gpsData);
    return;
  }
  command.trim(); // Remove whitespace or newline
  Serial.println(command);
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) return; // Invalid command format

  String dir = command.substring(0, commaIndex);
  int speed = command.substring(commaIndex + 1).toInt();

  if (dir == "f") {
    setMotorSpeeds(speed, speed);
  } else if (dir == "b") {
    setMotorSpeeds(-speed, -speed);
  } else if (dir == "r") {
    setMotorSpeeds(-speed, speed); // Adjust for turning
  } else if (dir == "l") {
    setMotorSpeeds(speed, -speed); // Adjust for turning
  } else if (dir == "al") {
    setMotorSpeeds(speed*0.6, speed); // Adjust left
  } else if (dir == "ar") {
    setMotorSpeeds(speed, speed*0.6); // Adjust right
  } else if (dir == "s") {
    setMotorSpeeds(0, 0); // Stop the car
  }
}

void setup() {
  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize SoftwareSerial
  Serial.begin(9600);
  gpsSerial.begin(9600);  // GPS baud rate

  // Serial.println("Arduino ready for commands.");
}

void loop() {
  if (Serial.available()) {
    // Serial.println("--------------------");
    String command = Serial.readStringUntil('\n'); // Read command from Serial
    // Serial.println(command);  // Send back the received command
    // Serial.println("Received: " + command);         // Echo for debugging
    handleCommand(command);                         // Process command
  }
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}
