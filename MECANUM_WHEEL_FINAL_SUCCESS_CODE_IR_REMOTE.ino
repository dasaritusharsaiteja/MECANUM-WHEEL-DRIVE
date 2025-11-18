#include <IRremote.hpp>

#define IR_RECEIVE_PIN 12  // IR Receiver Data Pin

// Motor Control Pins
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

IRrecv irrecv(IR_RECEIVE_PIN);

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

  analogWrite(EN_FL, 230);
  analogWrite(EN_FR, 230);
  analogWrite(EN_BL, 230);
  analogWrite(EN_BR, 230);

  irrecv.begin(IR_RECEIVE_PIN, false, 0);

  Serial.println("Mecanum IR Control Ready...");
}

void loop() {
  if (irrecv.decode()) {
    unsigned long irCode = irrecv.decodedIRData.decodedRawData;

    if (irCode != 0) {
      Serial.print("IR Code Received: 0x");
      Serial.println(irCode, HEX);

      if (irCode == 0xE718FF00) forward();
      else if (irCode == 0xAD52FF00) backward();
      else if (irCode == 0xF708FF00) strafeLeft();
      else if (irCode == 0xA55AFF00) strafeRight();
      else if (irCode == 0xBA45FF00) rotateLeft();
      else if (irCode == 0xB946FF00) rotateRight();
      else if (irCode == 0xB847FF00) diagFrontLeft();
      else if (irCode == 0xBB44FF00) diagFrontRight();
      else if (irCode == 0xBF40FF00) diagBackLeft();
      else if (irCode == 0xBC43FF00) diagBackRight();
      else if (irCode == 0xE31CFF00) stopMotors();
    }

    irrecv.resume();
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
