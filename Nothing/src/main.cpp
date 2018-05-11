#include "Arduino.h"
#include "../lib/Servo.h"


//#################################### Порты ###################################

#define port_motor_left_pwm        6
#define port_motor_left_dir        13

#define port_motor_right_pwm       5
#define port_motor_right_dir       12

#define port_sensor_front_left
#define port_sensor_front_right

#define port_sensor_left_front
#define port_sensor_left_back

#define port_sensor_right_front
#define port_sensor_right_back

#define port_sensor_back
#define port_sensor_cube

#define port_servo_up  9
#define port_servo_down

#define port_servo_lift  8
#define port_servo_claw  10

//################################# Обозначения ################################

#define M_LEFT                0
#define M_RIGHT               1
#define FORWARD               0
#define BACKWARD              1
#define FRONT                 0
#define BACK                  1
#define LEFT                  2
#define RIGHT                 3


//################################## Константы #################################

const char encoder_table[] = {0, -1, 1, 0, 1, 0, 0, -1,
                              -1, 0, 0, 1, 0, 1, -1, 0};

const float pid_motor_p = 1;                // Пропорционадьный;
const float pid_motor_d = 0;                // Деференциальный;
const float pid_motor_i = 0.005;            // Интегральный;
const float pid_motor_i_div = 1;            // Делитель интегрального;

const float motor_spd_mul_r = 1.64634;
const float motor_spd_mul_l = 1.63095;
const int motor_spd_add_r = 0;              // 15;
const int motor_spd_add_l = 0;              // 19;

// Коды запуска ядерных ракет;
// 0111 - стенка сверху
// 1101 - стенка снизу
// 1101 - стенка слева
// 1110 - стенка справа
const unsigned char map_sens[4][8] = {
  { 0b10111110, 0b11111101, 0b10111110, 0b11111101 },
  { 0b11111101, 0b10111110, 0b01111101, 0b11111111 },
  { 0b10111110, 0b11111101, 0b10111110, 0b10111101 },
  { 0b11111101, 0b10111110, 0b01111101, 0b11111111 },
  { 0b10111110, 0b11111101, 0b10111110, 0b10111101 },
  { 0b10111110, 0b10111101, 0b10111110, 0b11111101 },
  { 0b10111110, 0b11111101, 0b10111110, 0b11111101 },
  { 0b11111101, 0b11111110, 0b01111101, 0b11111111 }
};

const byte sensor_smoothing_length = 4;  // Длинна массива сглаживания сенсеров;

const int section_length = 1910;
const int rotate_90_length = 725;
const int rotate_180_length = 1450;

const int section_delta_length = 400;       // 350;
const int section_delta_free = 50;
const int move_forward_delta = (section_length / 2) - section_delta_length;

const int move_rotate90_delta = (rotate_90_length / 2) - section_delta_length;
const int move_rotate180_delta = (rotate_180_length / 2) - section_delta_length;

const byte wall_depth = 16;
const byte wall_range = 68;

//################################## Переменные моторов ########################

int motor_left;                             // Скорости моторов;
int motor_right;

int motor_left_pwm;                         // Энергия подаваемая на моторы;
int motor_right_pwm;

int max_target_spd;                         // Максимальня скорость моторов;

//################################## Переменные энкодеров ######################

volatile long encoder_count_l = 0, encoder_count_r = 0;   // Счетчики энкодеров;
volatile long encoder_last_count_l, encoder_last_count_r;
volatile long enc_spd_l, enc_spd_r;                        // Реальная скорость;

//################################## Переменные пидов ##########################

int target_spd, target_spd_l, target_spd_r;
int pid_l, pid_r;
int pid_p_l, pid_p_r;                                       // Пропорциональный;
int pid_last_p_l, pid_last_p_r, pid_d_l, pid_d_r;           // Дифференциальный;
long pid_i_l, pid_i_r;                                          // Интегральный;

float real_target_dir, enc_dir;
int target_dir;

//Синхронное PID регулирование моторов;
float pid_s;                                                     // Общий вывод;
float pid_p_s;                                // Пропорциональный для direction;
float pid_last_p_s, pid_d_s;                  // Дифференциальный для direction;
float pid_i_s;                                    // Интегральный для direction;

unsigned long timerPrint;

//################################## Переменные сенсоров #######################

//0 - передний сенсор, 1 - правый сенсор, 2 - задний сенсор, 3 - левый сенсор;


int sensor_front_right_var;                              // Значение сенсоров;
int sensor_front_left_var;

int sensor_right_front_var;
int sensor_right_back_var;

int sensor_left_front_var;
int sensor_left_back_var;

int sensor_back_var;

int sensor_r_smoothing_arr[sensor_smoothing_length];      // Массивы для сглаживания сенсоров;
int sensor_l_smoothing_arr[sensor_smoothing_length];

byte sensor_r_smoothing_count;                            // Счетчики для массивов сглаживания сенсоров;
byte sensor_l_smoothing_count;

byte sensor_r_smoothing_local_length = 0;
byte sensor_l_smoothing_local_length = 0;

//################################## Переменные локализации ################################

byte sens_real = 0;                 // Нынешние значения сенсоров;

byte map_possibly[60][6];           // *4
byte map_possibly_size;
byte map_possibly_count;

byte robot_i, robot_j;              // Нынешние координаты робота;
byte robot_dir;                     // Нынешнее направление робота;
byte start_i, start_j;              // Координаты старта;
byte fin_i, fin_j;                  // Координаты следующей клетки;
byte tmp_robot_dir;

bool isCubCathed = false;

byte offset_in_this_point = 0;
byte walls_in_this_point = 0;

byte range_wall_left = 0;
byte range_wall_right = 0;
byte old_range_wall_right = 0;
byte old_range_wall_left = 0;

char red_line[64][2];               // Массив координат;
byte line_len = 0;                  // Длина Массива;
byte line_dir = 0;                  // Текущее направление робота;
byte min_len = 255;
byte now_pos = 0;                   // Текущая точка;
char pos_i;
char pos_j;

//################################## Переменные поиска пути ####################
char lab_map[64][5] = { {1, 8, -1, -1, -1},
{0, 2, -1, -1, -1},
{1, 3, 10, -1, -1},
{2, 4, -1, -1, -1},
{3, 5, 12, -1, -1},
{4, 6, -1, -1, -1},
{5, 7, 14, -1, -1},
{6, -1, -1, -1, -1},
{0, 16, -1, -1, -1},
{10, -1, -1, -1, -1},
{2, 9, 18, -1, -1},
{12, -1, -1, -1, -1},
{4, 11, 20, -1, -1},
{14, -1, -1, -1, -1},
{6, 13, 22, -1, -1},
{-1, -1, -1, -1, -1},
{8, 24, -1, -1, -1},
{18, 25, -1, -1, -1},
{10, 17, 26, -1, -1},
{20, 27, -1, -1, -1},
{12, 19, 28, -1, -1},
{22, 29, -1, -1, -1},
{14, 21, 30, -1, -1},
{31, -1, -1, -1, -1},
{16, 25, -1, -1, -1},
{17, 24, 26, -1, -1},
{18, 25, 27, -1, -1},
{19, 26, 28, -1, -1},
{20, 27, 29, -1, -1},
{21, 28, 30, -1, -1},
{22, 29, 31, -1, -1},
{23, 30, -1, -1, -1},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0},
{0, 0, 0, 0, 0}};


byte que[64];                 // Очередь;
byte start_que = 0;           // Начало очереди;
byte end_que = 0;             // Конец очереди;

byte fin = 0;

byte last[64];

byte path[64];                // Путь;
byte path_size = 0;           // Длинна пути;

byte way_len[9] = { 0, 0, 0, 5, 8, 9, 8, 9, 5 };
byte point_pos[4] = { 23, 13, 9, 24 };
//r~1, r~2, r~3, 1~2, 2~3, 1~3, 1~s, 2~s, 3~s;
//1, 2, 3, s;


//################################## Переменные ускорения ################################

long move_count = 0;
long move_center = 0;

bool next_acceleration = true;
bool next_deacceleration = false;

bool forward_alignment_trigger = false;

//################################## Переменные захвата кубика ################################

byte cub_catching = 0; // Стадии принятия кубика
long cub_catching_timer;
byte claw_degr = 180;

byte robot_cord_now;

void setup() {
    if DEBUG {
      Serial.begin(115200);
      delay(2000)
      Serial.println("Start!\n")
      delay(2000)
    }

    encSetup();
    motorSetup();
    sensorsSetup();
    servoSetup();

    delay(100);

    target_dir = 0;

    updateSensVar(LEFT);
    updateSensVar(RIGHT);
    if (!((sensor_left_front_var < 300) &&
          (sensor_left_back_var < 300) &&
          (sensor_right_front_var < 300) &&
          (sensor_right_back_var < 300))){
            max_target_spd = 35;
            mainLocalize();
            max_target_spd = 80;
            mainPath();
          }


    target_spd = 0;
    target_dir = 0;
    motorWritePwm(M_LEFT, 0);
    motorWritePwm(M_RIGHT, 0);


}

void loop() {
    // put your main code here, to run repeatedly:
}
