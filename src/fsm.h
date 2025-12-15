#ifndef FSM_H
#define FSM_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
//#include <MPU6050.h>¨
#include <VectorXf.h>
#include "MPU6500_Raw.h"
#include <MadgwickAHRS.h>
#include "PID.h"

// analog pins for the line sensors:
#define IR1_pin 22
#define IR2_pin A0  //GPIO26
#define IR3_pin A1  //GPIO27
#define IR4_pin A2  //GPIO28
#define IR5_pin 19

// Encdoder input pins for the motors:
#define LEFT_MOTOR_ENCODER_A_pin 11
#define LEFT_MOTOR_ENCODER_B_pin 10
#define RIGHT_MOTOR_ENCODER_A_pin 9
#define RIGHT_MOTOR_ENCODER_B_pin 8

// Motor control pins:
#define LEFT_MOTOR_D0_pin 7
#define LEFT_MOTOR_D1_pin 6
#define RIGHT_MOTOR_D2_pin 5
#define RIGHT_MOTOR_D3_pin 4

// Laser ranging sensor I2C0 pins:
#define LASER_RANGING_SCL_pin 21
#define LASER_RANGING_SDA_pin 20 

// IMU I2C pins: I2C1
#define IMU_SCL_pin 14
#define IMU_SDA_pin 15

// ODOMETRY CONSTANTS - FROM ESCOBAR
// Encoder ticks per revolution
#define ENCODER_TICKS_PER_REV 1920u
// Wheel distance in mm
#define WHEEL_DISTANCE_M 0.105 // to be measured
// Wheel radius in mm
#define WHEEL_RADIUS_M 0.0689/2 // to be measured
// Maximum linear velocity in m/s
#define DV_MAX 5.0f
// Maximum angular velocity in rad/s
#define DW_MAX 10.0f
// Control loop time step in seconds
#define DT 0.04f
// Conversion factor from encoder ticks to distance in meters
#define CONVERSION_FACTOR_K    ((2.0f * 3.14159265f * WHEEL_RADIUS_M) / ENCODER_TICKS_PER_REV)




typedef struct {
  // IR sensor readings analog inputs
  uint16_t IR1, IR2, IR3, IR4, IR5;
  float central_distance;
} line_sensor_inputs_t;

typedef struct {
  // Motor control outputs
  int8_t left_motor_speed;   // -100 to 100
  int8_t right_motor_speed;  // -100 to 100
} motor_t;

// Ultrasound sensor input
typedef struct {
  uint16_t distance;
} ultrasound_sensor_input_t;

// PID controller structure
typedef struct {
    PID_t left;   // PID controller for the left wheel
    PID_t right;  // PID controller for the right wheel
} pid_controller_t;

// Odometry structure
typedef struct {
    int32_t left_wheel_ticks; // encoder ticks
    int32_t right_wheel_ticks;  // encoder ticks
    float dl,dr; // distance travelled by left and right wheels in one step
    float d; // distance travelled in one step
    float x;  // in mm
    float y;  // in mm
    float theta; // in radians
    float vl, vr; // estimated linear velocities of left and right wheels
    float wl, wr; // estimated angular velocities of left and right wheels
    float v;  // estimated linear velocity of robot
    float w;  // estimated angular velocity of robot

    float v_req;
    float w_req;
    float wl_req;
    float wr_req;
} odometry_t;

typedef struct{
  Adafruit_VL53L0X lox;
  VL53L0X_RangingMeasurementData_t measure;
  bool outOfRange;
  uint16_t distance;
}laser_ranging_sensor_t;

typedef struct{
  Vec3f w;
  Vec3f a;
  uint32_t cycle_time, last_cycle_time; // IMU cycle tracking
  float roll, pitch, yaw;  // Euler angles from Madgwick filter
}imu_t;


// Struct to hold the state for a finite state machine
class fsm
{
private:
  void readLineSensors();
  void readLaserRangingSensor();
  void readIMU();
  void readEncoders();

public:
  int state, new_state;
  // peripherals associated with this FSM
  line_sensor_inputs_t line_sensors;
  motor_t motors;
  ultrasound_sensor_input_t ultrasound_sensor;
  pid_controller_t velocity_pid_controller;
  pid_controller_t position_pid_controller;
  odometry_t odometry;
  laser_ranging_sensor_t laser_ranging_sensor;
  imu_t imu;
  MPU6500 mpu;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;

  fsm();
  ~fsm();
  void init();
  void setState(int new_state);
  void updateTisTes();
  void calibrateIMU();
  void calculateOdometry();
  void readSensors();
  void updateVelocityPID();
  void setVelocityReferences(float v_req, float w_req);
  void writeOutputs();

};

// meaningful names for the fsm1 states
enum {
  IDLE = 0,
  CALIBRATION_IMU = 1,
  LINE_FOLLOW = 2
};


#endif // FSM_H
