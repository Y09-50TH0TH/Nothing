#include "main.cpp"

void motorSetup(){
  pinMode(port_motor_left_pwm OUTPUT);
  pinMode(port_motor_left_dir, OUTPUT);

  pinMode(port_motor_right_pwm, OUTPUT);
  pinMode(port_motor_right_dir, OUTPUT);
}

void motorWriteDir(byte motor, byte dir) {
  if (motor == M_LEFT) {
    digitalWrite(port_motor_left_dir, dir);
  }
  if (motor == M_RIGHT) {
    digitalWrite(port_motor_right_dir, dir);
  }
}

void motorWritePwm(byte motor, byte power) {
  if (motor == M_LEFT) {
    analogWrite(port_motor_left_pwm, power);
  }
  if (motor == M_RIGHT) {
    analogWrite(port_motor_right_pwm, power);
  }
}

void motorWriteDirPwm(byte motor, byte dir, byte power) {
  motorWriteDir(motor, dir);
  motorWritePwm(motor, power);
}

void motorWrite(byte motor, int power) {
  if (power > 0) {
    motorWriteDirPwm(motor, FORWARD, power);
  } else {
    motorWriteDirPwm(motor, BACKWARD, abs(power));
  }
}

void motorSpeedToPwm() {

  if (motor_left == 0){
    motor_left_pwm = 0;
  } else {
    if (motor_left > 0){
      motor_left_pwm = (motor_left *motor_spd_mul_l) + motor_spd_add_l;
      if (motor_left_pwm > 255) {
        motor_left_pwm = 255;
      }
    } else {
      motor_left_pwm = (motor_left * motor_spd_mul_l) - motor_spd_add_l;
      if (motor_left_pwm < -255) {
        motor_left_pwm = -255
      }
    }
  }

  if (motor_right == 0) {
    motor_right_pwm = 0;
  } else {
    if (motor_right > 0) {
      motor_right_pwm = (motor_right * motor_spd_mul_r) + motor_spd_add_r;
      if (motor_right_pwm > 255) {
        motor_right_pwm = 255;
      }
    } else {
      motor_right_pwm = (motor_right * motor_spd_mul_r) - motor_spd_add_r;
      if (motor_right_pwm < -255) {
        motor_right_pwm = -255;
      }
    }
  }
}

void motorBoth() {
  motorSpeedToPwm();
  motorWrite(M_LEFT, motor_left_pwm);
  motorWrite(M_RIGHT, motor_right_pwm);
}

void motorPause() {
  target_spd_l = 0;
  target_spd_r = 0;
  pidMotorPoth();
  motorBoth();
}
