
#include <Wire.h>
#include "KB_Motor.h"



KB_MOTOR i2c_motor; //Create a new KB_MOTOR instance

uint8_t motor1_addr = 0x66; //i2c Address Motor Control Motor 1
uint8_t motor2_addr = 0x68; //i2c Address Motor Control Motor 2

int L1, R1;

void setup() {
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  Wire.begin();
  // put your setup code here, to run once:
  i2c_motor.begin(motor1_addr, motor2_addr); //init Motor Module 2CH
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  L1 = digitalRead(32);
  R1 = digitalRead(33);
  Serial.print("L");
  Serial.println(digitalRead(32));

  Serial.print("R");
  Serial.println(digitalRead(33));

  if (L1 == 1 && R1 == 1) {
    i2c_motor.i2c_motor_write(1, 125, 1);
    i2c_motor.i2c_motor_write(2, 125, 1);
  }
  else if (L1 == 1 && R1 == 0)
  {
    i2c_motor.i2c_motor_write(1, 125, 1);
    i2c_motor.i2c_motor_write(2, 255, 1);
  }
  else if (L1 == 0 && R1 == 1)
  {
    i2c_motor.i2c_motor_write(1, 255, 1);
    i2c_motor.i2c_motor_write(2, 125, 1);
  }

  // { stat (0 = stop) ,(1 = Forward) ,(2 = Backward) }
  //  i2c_motor.i2c_motor_write(1, 255, 1);
  //  i2c_motor.i2c_motor_write(2, 255, 1);

  delay(100);

}
