#include "main.cpp"

void sensorsSetup() {
  pinMode(port_sensor_back, OUTPUT)
  pinMode(port_sensor_front_left, OUTPUT)
  pinMode(port_sensor_front_right, OUTPUT)
  pinMode(port_sensor_left_front, OUTPUT)
  pinMode(port_sensor_left_back, OUTPUT)
  pinMode(port_sensor_right_front, OUTPUT)
  pinMode(port_sensor_right_back, OUTPUT)
}

void updateSensVar(int n) {
  switch (n) {
    case 0:
      sensor_front_right_var = analogRead(port_sensor_front_right)
      sensor_front_left_var = analogRead(port_sensor_left_right)
    case 1:
      sensor_back_var = analogRead(port_sensor_back)
    case 2:
      sensor_left_front_var = analogRead(port_sensor_left_front)
      sensor_left_back_var = analogRead(port_sensor_left_back)
    case 3:
      sensor_right_front_var = analogRead(port_sensor_right_front)
      sensor_right_back_var = analogRead(port_sensor_right_back)

  }
}
