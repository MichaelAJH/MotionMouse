
// include libraries
#include "FastIMU.h"
#include <Wire.h>

// loop control speed
#define RATE 10 // Hz
#define LOOP 1000000.0 / RATE // microsecs
uint32_t timer; // microsecs

// button pinout
#define P1_PIN 14
#define P2_PIN 32
#define P3_PIN 33
#define Reset_PIN 12

// variables for button
bool ButtonState_P1 = HIGH;
bool ButtonState_P2 = HIGH;
bool ButtonState_P3 = HIGH;
bool ButtonState_Reset = HIGH;
float long_press = 300000; // ms
float tolerance = 100000;  // ms

// variables for update_P1
int state = 0;
float press_start_time;
float check_time;
int temp_state = 0;
int click_state = 0;
int long_state = 0;
int scroll_state = 0;
int move_state = 0;

// IMU stuff
#define IMU_ADDRESS 0x68
MPU9255 imu;
calData calib = {0};
AccelData accel;
GyroData gyro;
float accel_tolerance=0.05;
float gyro_tolerance=1;
float vel_x_tolerance=0.0;
float vel_y_tolerance=0.0;
float vel_z_tolerance=0.0;
double Vx = 0;
double Vy = 0;
double Vz = 0;
double Tx = 0;

// Calibration and Adjustment
#define CALIBRATE true
int x_length = 1920;
int y_length = 1080;
int check_state = 0;
double a[3] = {0,0,0};
double b[3] = {0,0,0};
float size_a;
float size_b;
double X[3] = {0,0,0};
double Y[3] = {0,0,0};
float size_X=0;
float size_Y=0;
double inner;
double k;

// functions
double denoise(double value, char c) {
  float tolerance;
  if( c == 'a' ) tolerance = accel_tolerance;
  else if( c == 'g' ) tolerance = gyro_tolerance;
  else if( c == 'x' ) tolerance = vel_x_tolerance;
  else if( c == 'y' ) tolerance = vel_y_tolerance;
  else if( c == 'z' ) tolerance = vel_z_tolerance;

  if (-1*tolerance < value && value < tolerance) return 0;
  else return value;
}

double integral(double* sum, double value) {
  *sum += value * ( LOOP / 1000000.0 );
  return *sum;
}

void adjust_vel(double prev, double* vel) {
  if( prev == *vel ) *vel = 0.0;
}

void pre_run() {
  bool need_calibration = true;
  bool need_adjustment = false;
  bool need_x_scaling = true;
  bool need_y_scaling = true;

  while( need_calibration ) {
    if( Serial.available() > 0 && Serial.read() == 'C') {
      Serial.println("Starting calibration");
      delay(5000);
      imu.calibrateAccelGyro(&calib);
      imu.init(calib, IMU_ADDRESS);
      Serial.println("Calibrated");
      need_calibration = false;
    }
  }
  Serial.println("X axis set");
  Serial.println("Y axis set");
  while( need_adjustment ) {
      if( Serial.available() > 0 && Serial.read() == 'S') {
        // Serial.println("Starting scaling");
        float x = 0;
        float y = 0;
        float z = 0;
        while( need_x_scaling ) {
          bool ButtonState_P3 = digitalRead(P3_PIN);
          if( check_state == 0 && ButtonState_P3 == LOW ) {
            check_state = 1;
          } else if( check_state == 1 && ButtonState_P3 == HIGH ){
            // Serial.println("Released");
            check_state = 2;
          } else if( check_state == 2 && ButtonState_P3 == LOW ) {
            // Serial.println("Pressed again");
            check_state = 0;
            a[0] = x;
            a[1] = y;
            a[2] = z;
            Serial.println("X axis set");
            // Serial.printf("%f, %f, %f\n", a[0], a[1], a[2]);
            need_x_scaling = false;
          }
          if( check_state == 1 ){
            imu.update();
            imu.getAccel(&accel);
            float Ax = denoise(accel.accelX, 'a');
            float Ay = denoise(accel.accelY, 'a');
            float Az = denoise(accel.accelZ - 1, 'a');
            Vx = denoise(integral(&Vx, Ax), 'x');
            Vy = denoise(integral(&Vy, Ay), 'y');
            Vz = denoise(integral(&Vy, Az), 'z');
            x += Vx;
            y += Vy;
            z += Vz;
          }
          while( micros() - timer < LOOP );
          timer = micros();
        }

        x = 0;
        y = 0;
        z = 0;
        delay(500);
        while( need_y_scaling ) {
          bool ButtonState_P3 = digitalRead(P3_PIN);
          if( check_state == 0 && ButtonState_P3 == LOW ) {
            check_state = 1;
          } else if( check_state == 1 && ButtonState_P3 == HIGH ){
            // Serial.println("Released");
            check_state = 2;
          } else if( check_state == 2 && ButtonState_P3 == LOW ) {
            // Serial.println("Pressed again");
            check_state = 0;
            b[0] = x;
            b[1] = y;
            b[2] = z;
            Serial.println("Y axis set");
            // Serial.printf("%f, %f, %f\n", b[0], b[1], b[2]);
            need_y_scaling = false;
          }
          if( check_state == 1 ){
            imu.update();
            imu.getAccel(&accel);
            float Ax = denoise(accel.accelX, 'a');
            float Ay = denoise(accel.accelY, 'a');
            float Az = denoise(accel.accelZ - 1, 'a');
            Vx = denoise(integral(&Vx, Ax), 'x');
            Vy = denoise(integral(&Vy, Ay), 'y');
            Vz = denoise(integral(&Vy, Az), 'z');
            x += Vx;
            y += Vy;
            z += Vz;
          }
          while( micros() - timer < LOOP );
          timer = micros();
        }
        need_adjustment = false;
      }
  }

  size_a = pow((pow(a[0],2) + pow(a[1],2) + pow(a[2],2)), 0.5);
  size_b = pow((pow(b[0],2) + pow(b[1],2) + pow(b[2],2)), 0.5);
  X[0] = (x_length / size_a) * a[0];
  X[1] = (x_length / size_a) * a[1];
  X[2] = (x_length / size_a) * a[2];
  Y[0] = (y_length / size_b) * b[0];
  Y[1] = (y_length / size_b) * b[1];
  Y[2] = (y_length / size_b) * b[2];
  size_X = pow((pow(X[0],2) + pow(X[1],2) + pow(X[2],2)), 0.5);
  size_Y = pow((pow(Y[0],2) + pow(Y[1],2) + pow(Y[2],2)), 0.5);
  inner = X[0]*Y[0] + X[1]*Y[1] + X[2]*Y[2];
  k = -1 * inner / x_length;
  Serial.println("Adjusted");
}

void update_P1(bool ButtonState_P1){
  switch(state) {
    case 0: {
      click_state = 0;
      long_state=0;
      if(ButtonState_P1==LOW) {
        state=1;
        press_start_time=micros();
        temp_state++;
      }
    } break;
    case 1: {
      if(ButtonState_P1==HIGH) {
        state=3;
        check_time=micros();
      } else if( micros()-press_start_time > long_press && ButtonState_P1==LOW ){
        state=2;
        temp_state=0;
        long_state=1;
      }
    } break;
    case 2: {
      if(ButtonState_P1==HIGH) {
        state=3;
        check_time=micros();
      } break;
    }
    case 3: {
      if( micros()-check_time < tolerance && ButtonState_P1==LOW ) {
        if( click_state < 2 ) {
          state=1;
          press_start_time=micros();
          temp_state++;
        } else {
          state=0;
          temp_state=0;
          long_state=0;
        }
      } else if( micros()-check_time > tolerance ) {
        state=0;
        click_state=temp_state;
        temp_state=0;
      } break;
    }
  }
}

void update_P2(bool ButtonState_P2) {
  if( ButtonState_P2==LOW ) scroll_state=1;
  else scroll_state=0;
}

void update_P3(bool ButtonState_P3) {
  if( ButtonState_P3==LOW ) move_state=1;
  else move_state=0;
}

double get_p(double r1, double r2, double r3) {
  float s1 = (r1*X[0] + r2*X[1] + r3*X[2]) / pow(size_X,2);
  float s2 = (k*(r1*Y[0] + r2*Y[1] + r3*Y[2]) + pow(k,2)*(r1*X[0] + r2*X[1] + r3*X[2])) / (pow(size_X,2) + 2*k*inner + pow(size_Y,2));
  return s1 + s2;
}

double get_q(double r1, double r2, double r3) {
  return (r1*Y[0] + r2*Y[1] + r3*Y[2] + k*(r1*X[0] + r2*X[1] + r3*X[2])) / (pow(size_X,2) + 2*k*inner + pow(size_Y,2));
}

// setup: serial connection, calibration of IMU, scale adjustment to screen
void setup() {
  // wire, clock, serial, pins
  Wire.setPins(27, 26);
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(1000);
  pinMode(P1_PIN, INPUT_PULLUP);
  pinMode(P2_PIN, INPUT_PULLUP);
  pinMode(P3_PIN, INPUT_PULLUP);
  pinMode(Reset_PIN, INPUT_PULLUP);

  // in case of error
  int error = imu.init(calib, IMU_ADDRESS);
  if( error != 0 ) {
    Serial.println("Error initializing IMU ):");
    Serial.println(error);
    while( true );
  }

  pre_run();
  // Serial.println("pre_run complete");
  timer = micros();
}

void loop() {
  bool ButtonState_P1 = digitalRead(P1_PIN);
  bool ButtonState_P2 = digitalRead(P2_PIN);
  bool ButtonState_P3 = digitalRead(P3_PIN);
  bool ButtonState_Reset = digitalRead(Reset_PIN);
  if( ButtonState_Reset == LOW ) {
    Vx = 0;
    Vy = 0;
    Vz = 0;
    Tx = 0;
  }
  update_P1(ButtonState_P1);
  update_P2(ButtonState_P2);
  update_P3(ButtonState_P3);

  imu.update();
  imu.getAccel(&accel); // accelerometer data
  imu.getGyro(&gyro);   // gyroscope data
  double Ax = denoise(accel.accelX, 'a');
  double Ay = denoise(accel.accelY, 'a');
  double Az = denoise(accel.accelZ - 1, 'a');
  double Gx = denoise(gyro.gyroX, 'g');
  // Vx += denoise((Ax * (LOOP / 1000)), 'x');
  double prev_x = Vx;
  double prev_y = Vy;
  double prev_z = Vz;
  Vx = denoise(integral(&Vx, Ax), 'x');
  Vy = denoise(integral(&Vy, Ay), 'y');
  Vz = denoise(integral(&Vy, Az), 'z');
  adjust_vel(prev_x, &Vx);
  adjust_vel(prev_y, &Vy);
  adjust_vel(prev_z, &Vz);
  Tx = integral(&Tx, Gx);

  double p = get_p(Vx, Vy, Vz);
  double q = get_q(Vx, Vy, Vz);
  // Serial.printf("%d, %d, %d, %d, %d, %f, %f, %f, %f, %d\n", state, click_state , long_state , scroll_state, move_state, Vx, Vy, Vz, Gx, micros()-timer);
  // Serial.printf("%d %d %d %d %f %f %f %f\n", click_state , long_state , scroll_state, move_state, size_X, p, q, Tx);
  Serial.printf("%d %d %d %d %f %f %f\n", click_state , long_state , scroll_state, move_state, Vx, Vy, Tx);

  while( micros() - timer < LOOP );
  timer = micros();
}
