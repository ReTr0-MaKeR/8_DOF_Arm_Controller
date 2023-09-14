/*******************************************************
 * Project Title:Dual Arm Control 
 * Author: Md. Tanvir Hassan
 * Inspired By/Idea Taken From: Pinaut
 * 
 * Description:
 * This Arduino sketch controls a set of mini robot arms using potentiometers to read sensor values
 * and servo motors to move the arms. The code allows you to record and playback arm movements.
 * 
 * Hardware Requirements:
 * - Arduino board
 * - Servo motors (8 in total, divided into two sets)
 * - Potentiometers (8 in total, divided into two sets)
 * - Pushbuttons (optional for control)
 * - Other necessary components as per your hardware setup

 Warning This Code works only for Arduino mega or higher memory AVR boards 
 
 *******************************************************/
 


#include <Servo.h>

Servo servoArm1_0;
Servo servoArm1_1;
Servo servoArm1_2;
Servo servoArm1_3;

Servo servoArm2_0;
Servo servoArm2_1;
Servo servoArm2_2;
Servo servoArm2_3;

int sensorPinArm1_0 = A0;
int sensorPinArm1_1 = A1;
int sensorPinArm1_2 = A2;
int sensorPinArm1_3 = A3;

int sensorPinArm2_0 = A4;
int sensorPinArm2_1 = A5;
int sensorPinArm2_2 = A6;
int sensorPinArm2_3 = A7;

int count0, arrayStep, arrayMax, countverz, Taster, stepsMax, steps, time = 1000, del = 1000, temp;
unsigned int verz = 0;

long previousMillis1 = 0;
long previousMillis2 = 0;
long previousMillis3 = 0;
long previousMillis4 = 0;
long previousMicros = 0;
unsigned long currentMillis = millis();
unsigned long currentMicros = micros();

int Delay[7] = {0, 0, 1, 3, 15, 60, 300};
int SensValArm1[4];
int SensValArm2[4];
float difArm1[4], istArm1[4], solArm1[4], dirArm1[4];
float difArm2[4], istArm2[4], solArm2[4], dirArm2[4];
int jointArm1_0[180];
int jointArm1_1[180];
int jointArm1_2[180];
int jointArm1_3[180];
int jointArm2_0[180];
int jointArm2_1[180];
int jointArm2_2[180];
int jointArm2_3[180];

int top = 179;

boolean playmode = false, Step = false;

void setup() {
  pinMode(4, INPUT);
  pinMode(6, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  servoArm1_0.attach(3);
  servoArm1_1.attach(10);
  servoArm1_2.attach(9);
  servoArm1_3.attach(11);
  
  servoArm2_0.attach(5);
  servoArm2_1.attach(6);
  servoArm2_2.attach(7);
  servoArm2_3.attach(8);

  Serial.begin(115200);
  Serial.println("Mini Robot Arms ready...");
  digitalWrite(13, LOW);
}

void loop() {
  currentMillis = millis();
  currentMicros = micros();
  Button();

  if (!playmode) {
    if (currentMillis - previousMillis1 > 25) {
      if (arrayStep < top) {
        previousMillis1 = currentMillis;
        readPot();
        mapping();
        move_servo();
      }
    }
  } else if (playmode) {
    if (Step) {
      digitalWrite(13, HIGH);
      if (arrayStep < arrayMax) {
        arrayStep += 1;
        Read();
        calculate();
        Step = false;
        digitalWrite(13, LOW);
      } else {
        arrayStep = 0;
        calc_pause();
        countverz = 0;
        while (countverz < verz) {
          countverz += 1;
          calc_pause();
          digitalWrite(13, HIGH);
          delay(25);
          digitalWrite(13, LOW);
          delay(975);
        }
      }
    } else {
      if (currentMicros - previousMicros > time) {
        previousMicros = currentMicros;
        play_servo();
      }
    }
  }

  while (digitalRead(4) == false) {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}

void calc_pause() {
  readPot();
  temp = SensValArm1[3];
  if (temp < 0) temp = 0;
  temp = map(temp, 0, 680, 0, 6);
  verz = Delay[temp];
}

void readPot() {
  SensValArm1[0] = analogRead(sensorPinArm1_0);
  SensValArm1[1] = analogRead(sensorPinArm1_1);
  SensValArm1[2] = analogRead(sensorPinArm1_2);
  SensValArm1[3] = analogRead(sensorPinArm1_3);

  SensValArm2[0] = analogRead(sensorPinArm2_0);
  SensValArm2[1] = analogRead(sensorPinArm2_1);
  SensValArm2[2] = analogRead(sensorPinArm2_2);
  SensValArm2[3] = analogRead(sensorPinArm2_3);

  Serial.print(SensValArm1[2]);
  Serial.print(" ");
  Serial.print(SensValArm2[2]);
  Serial.print(" ");
}

void mapping() {
  for (int i = 0; i < 4; i++) {
    istArm1[i] = map(SensValArm1[i], 150, 900, 600, 2400);
    istArm2[i] = map(SensValArm2[i], 150, 900, 600, 2400);
  }
}

void record() {
  jointArm1_0[arrayStep] = istArm1[0];
  jointArm1_1[arrayStep] = istArm1[1];
  jointArm1_2[arrayStep] = istArm1[2];
  jointArm1_3[arrayStep] = istArm1[3];

  jointArm2_0[arrayStep] = istArm2[0];
  jointArm2_1[arrayStep] = istArm2[1];
  jointArm2_2[arrayStep] = istArm2[2];
  jointArm2_3[arrayStep] = istArm2[3];
}

void Read() {
  solArm1[0] = jointArm1_0[arrayStep];
  solArm1[1] = jointArm1_1[arrayStep];
  solArm1[2] = jointArm1_2[arrayStep];
  solArm1[3] = jointArm1_3[arrayStep];

  solArm2[0] = jointArm2_0[arrayStep];
  solArm2[1] = jointArm2_1[arrayStep];
  solArm2[2] = jointArm2_2[arrayStep];
  solArm2[3] = jointArm2_3[arrayStep];
}

void move_servo() {
  servoArm1_0.writeMicroseconds(istArm1[0]);
  servoArm1_1.writeMicroseconds(istArm1[1]);
  servoArm1_2.writeMicroseconds(istArm1[2]);
    servoArm1_3.writeMicroseconds(istArm1[3]);
  
  servoArm2_0.writeMicroseconds(istArm2[0]);
  servoArm2_1.writeMicroseconds(istArm2[1]);
  servoArm2_2.writeMicroseconds(istArm2[2]);
  servoArm2_3.writeMicroseconds(istArm2[3]);
}

void calculate() {
  for (int i = 0; i < 4; i++) {
    difArm1[i] = abs(istArm1[i] - solArm1[i]);
    difArm2[i] = abs(istArm2[i] - solArm2[i]);
  }

  stepsMax = max(difArm1[0], difArm1[1]);
  stepsMax = max(stepsMax, difArm1[2]);
  stepsMax = max(stepsMax, difArm1[3]);
  stepsMax = max(stepsMax, difArm2[0]);
  stepsMax = max(stepsMax, difArm2[1]);
  stepsMax = max(stepsMax, difArm2[2]);
  stepsMax = max(stepsMax, difArm2[3]);

  if (stepsMax < 500)
    del = 1200;
  else
    del = 600;

  for (int i = 0; i < 4; i++) {
    if (solArm1[i] < istArm1[i])
      dirArm1[i] = 0 - difArm1[i] / stepsMax;
    else
      dirArm1[i] = difArm1[i] / stepsMax;

    if (solArm2[i] < istArm2[i])
      dirArm2[i] = 0 - difArm2[i] / stepsMax;
    else
      dirArm2[i] = difArm2[i] / stepsMax;
  }
}

void play_servo() {
  steps += 1;
  if (steps < stepsMax) {
    if (steps == 20) time = del * 4;
    else if (steps == 40) time = del * 3;
    else if (steps == 80) time = del * 2;
    else if (steps == 100) time = del - 1;

    if (steps == stepsMax - 200) time = del * 2;
    else if (steps == stepsMax - 80) time = del * 3;
    else if (steps == stepsMax - 40) time = del * 4;
    else if (steps == stepsMax - 20) time = del * 5;

    for (int i = 0; i < 4; i++) {
      istArm1[i] += dirArm1[i];
      istArm2[i] += dirArm2[i];
    }

    servoArm1_0.writeMicroseconds(istArm1[0]);
    servoArm1_1.writeMicroseconds(istArm1[1]);
    servoArm1_2.writeMicroseconds(istArm1[2]);
    servoArm1_3.writeMicroseconds(istArm1[3]);

    servoArm2_0.writeMicroseconds(istArm2[0]);
    servoArm2_1.writeMicroseconds(istArm2[1]);
    servoArm2_2.writeMicroseconds(istArm2[2]);
    servoArm2_3.writeMicroseconds(istArm2[3]);
  } else {
    Step = true;
    steps = 0;
  }
}

void data_out() {
  int i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm1_0[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm1_0");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm1_1[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm1_1");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm1_2[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm1_2");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm1_3[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm1_3");

  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm2_0[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm2_0");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm2_1[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm2_1");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm2_2[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm2_2");
  i = 0;
  while (i < arrayMax) {
    digitalWrite(13, HIGH);
    i += 1;
    Serial.print(jointArm2_3[i]);
    Serial.print(", ");
  }
  Serial.println("JointArm2_3");
}

void Button() {
  if (digitalRead(6) == false) {
    delay(1);
    if (digitalRead(6) == true) {
      if (Taster == 0) {
        Taster = 1;
        previousMillis3 = currentMillis;
      } else if ((Taster == 1) && (currentMillis - previousMillis3 < 250)) {
        Taster = 2;
      }
    }
  }
    
  if ((Taster == 1) && (currentMillis - previousMillis3 > 1000)) {
    arrayStep += 1;
    arrayMax = arrayStep;
    record();
    Taster = 0;
    playmode = false;
    Serial.print("Record Step: ");
    Serial.println(arrayStep);
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
  } else if (Taster == 2) {
    arrayStep = 0;
    playmode = true;
    Taster = 0;
    Step = true;
    Serial.println("playmode ");
    data_out();
    delay(250);
    digitalWrite(13, LOW);
  }
    
  if (currentMillis - previousMillis3 > 2000) {
    Taster = 0;
  }
}

 
