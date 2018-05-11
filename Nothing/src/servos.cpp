#include "../lib/Servo.h"
#include "main.cpp"


void servoSetup() {
  servo_up.attach(port_servo_up);
  servo_down.attach(port_servo_down);
  servo_claw.attach(port_servo_claw);
  servo_lift.attach(port_servo_lift);
}

void rotateServo90(Servo servo) {
  //TODO Подогнать повороты
}


void catchcube() {
  if (cube_catching == 1) {
    cube_catching_timer = millis();
    claw_degr = 0;
    servo_claw.write(map(claw_degr, 0, 180, 1700, 2150));
    if (claw_degr == 0) {
      claw_degr = 180;
    }
  }
  else if (cube_catching == 2) {
    if ((claw_degr > 0) && ((millis() - cube_catching_timer) > 2)) {
      cube_catching_timer = millis();
      claw_degr--;
      servo_lift.write(map(claw_degr, 0, 180, 550, 2500));
    }
    if (claw_degr == 0) {
      cube_catching = 3;
    }
  }
  else if (cube_catching == 3) {
    if ((claw_degr < 180) && ((millis() - cube_catching_timer) > 2)) {
      cube_catching_timer = millis();
      claw_degr++;
      servo_claw.write(map(claw_degr, 0, 180, 1700, 2150));
    }
    if (claw_degr == 180) {
      cube_catching = 4;
      claw_degr = 0;
    }
  }
  else if (cube_catching == 4) {
    if ((claw_degr < 180) && ((millis() - cube_catching_timer) > 2)) {
      cube_catching_timer = millis();
      claw_degr++;
      servo_lift.write(map(claw_degr, 0, 180, 550, 2500));
    }
    if (claw_degr == 180) {
      cube_catching = 5;
    }
  } else if (cube_catching == 5) {
    switch (cubes_amount) {
      case 0:
        cubes_amount++;
        break;
      case 1:
        rotateServo90(servo_up);
        cubes_amount++;
        break;
      case 2:
        rotateServo90(servo_down);
        rotateServo90(servo_up);
    }
  }
}
