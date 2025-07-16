// Motor pins
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;

// IR sensor pins
const int leftIR = 7;
const int rightIR = 8;

void setup() {
  // Motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor pins as input
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  // Optional: Serial Monitor
  Serial.begin(9600);
}

void loop() {
  int leftSensor = digitalRead(leftIR);
  int rightSensor = digitalRead(rightIR);

  Serial.print("Left: ");
  Serial.print(leftSensor);
  Serial.print(" | Right: ");
  Serial.println(rightSensor);

  if (leftSensor == 0 && rightSensor == 0) {
    // Both sensors on black line - go forward
    moveForward();
  }
  else if (leftSensor == 0 && rightSensor == 1) {
    // Left on black, right on white - turn left
    turnLeft();
  }
  else if (leftSensor == 1 && rightSensor == 0) {
    // Right on black, left on white - turn right
    turnRight();
  }
  else {
    // Both on white - stop or move backward
    stopRobot();
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
