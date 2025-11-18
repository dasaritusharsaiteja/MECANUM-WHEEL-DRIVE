#define FL_IN1 30
#define FL_IN2 31
#define FR_IN1 24
#define FR_IN2 25
#define BL_IN1 28
#define BL_IN2 29
#define BR_IN1 26
#define BR_IN2 27

#define EN_FL 2
#define EN_FR 3
#define EN_BL 4
#define EN_BR 5

void setup() {
  Serial.begin(9600);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(BL_IN1, OUTPUT);
  pinMode(BL_IN2, OUTPUT);
  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);

  pinMode(EN_FL, OUTPUT);
  pinMode(EN_FR, OUTPUT);
  pinMode(EN_BL, OUTPUT);
  pinMode(EN_BR, OUTPUT);

  analogWrite(EN_FL, 200);
  analogWrite(EN_FR, 200);
  analogWrite(EN_BL, 200);
  analogWrite(EN_BR, 200);

  Serial.println("Mecanum Serial Control Ready...");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    Serial.print("Command received: ");
    Serial.println(command);

    switch (command) {
      case 'F': forward(); break;
      case 'B': backward(); break;
      case 'L': strafeLeft(); break;
      case 'R': strafeRight(); break;
      case 'Z': rotateLeft(); break;
      case 'X': rotateRight(); break;
      case '1': diagFrontLeft(); break;
      case '2': diagFrontRight(); break;
      case '3': diagBackLeft(); break;
      case '4': diagBackRight(); break;
      case 'S': stopMotors(); break;
      default: 
        Serial.println("Unknown command");
    }
  }
}

void forward() {
  Serial.println("Moving Forward");
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

void backward() {
  Serial.println("Moving Backward");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void strafeLeft() {
  Serial.println("Strafing Left");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void strafeRight() {
  Serial.println("Strafing Right");
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

void rotateLeft() {
  Serial.println("Rotating Left");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

void rotateRight() {
  Serial.println("Rotating Right");
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void diagFrontLeft() {
  Serial.println("Diagonal Front Left");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
}

void diagFrontRight() {
  Serial.println("Diagonal Front Right");
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

void diagBackLeft() {
  Serial.println("Diagonal Back Left");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void diagBackRight() {
  Serial.println("Diagonal Back Right");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, LOW);
}
