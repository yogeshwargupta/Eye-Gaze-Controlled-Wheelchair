// Define motor pins
#define MOTOR1_PIN1 2  // Motor 1 forward
#define MOTOR1_PIN2 3  // Motor 1 backward
#define MOTOR2_PIN1 4  // Motor 2 forward
#define MOTOR2_PIN2 5  // Motor 2 backward

// Variable to store the current direction
String currentDirection = "";

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available() > 0) {
    currentDirection = Serial.readStringUntil('\n');  // Read direction from Python script
    currentDirection.trim();  // Remove any extra whitespace or newline characters
    // Continuously execute the current direction
  if (currentDirection == "FORWARD") {
    moveForward();
  } else if (currentDirection == "BACK") {
    moveBackward();
  } else if (currentDirection == "LEFT") {
    turnLeft();
  } else if (currentDirection == "RIGHT") {
    turnRight();
  } else if (currentDirection == "STOP") {
    stopMotors();
  }
  }

  

 
}

// Function to move forward
void moveForward() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
  delay(3000);
   digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Function to move backward
void moveBackward() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
    delay(3000);
   digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Function to turn left
void turnLeft() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
    delay(3000);
   digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Function to turn right
void turnRight() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, HIGH);
    delay(3000);
   digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}
