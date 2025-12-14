#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>


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
// CHYBI MI KABEL NA NAPAJENI H-BRIDGE!!!

// IMU I2C pins: I2C1
#define IMU_SCL_pin 14
#define IMU_SDA_pin 15




// ODOMETRY CONSTANTS
// Wheel diameter in mm
#define CONVERSION_FACTOR_K  0.0f // to be measured -> d=k*ticks
// Encoder ticks per revolution
#define ENCODER_TICKS_PER_REV 1920u
// Wheel distance in mm
#define WHEEL_DISTANCE__FACTOR_MM 0.0f // to be measured




// ziskat konstanty k a b z měření podle jeho schematu
// kd nema smysl pro smysl 1. radu (motor) -> zpusobi problemy pri sumu

// control of speed -> PI
  // zero error s Pckem znamena zero voltage -> protoze integral
  // integral: Si = Si dt * error (dt bude ten malej cycle)
// control of position -> PD
  // NO OVERSHOOT

  

typedef struct {
  // IR sensor readings analog inputs
  uint16_t IR1, IR2, IR3, IR4, IR5;
  float central_distance;
} line_sensor_inputs_t;

typedef struct {
  // Motor control outputs
  int8_t left_motor_speed;   // -100 to 100
  int8_t right_motor_speed;  // -100 to 100

  // Motor encoder inputs
  // 1920 pulses per revolution
  uint16_t left_motor_encoder;
  uint16_t right_motor_encoder;
} motor_t;

// Ultrasound sensor input
typedef struct {
  uint16_t distance;
} ultrasound_sensor_input_t;

// PID controller structure
typedef struct {
  float kp;           // Proportional gain
  float ki;           // Integral gain
  float kd;           // Derivative gain
  float previous_error; // Previous error value
  float integral;      // Integral of the error
} pid_controller_t;

// Odometry structure
typedef struct {
  int32_t left_wheel_ticks; // encoder ticks
  int32_t right_wheel_ticks;  // encoder ticks
  float d1,d2; // distance travelled by left and right wheels in one step
  float d; // distance travelled in one step
  float x;  // in mm
  float y;  // in mm
  float theta; // in radians
} odometry_t;

typedef struct{
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  VL53L0X_RangingMeasurementData_t measure;
  bool outOfRange=0;
  uint16_t distance;
}laser_ranging_sensor_t;

typedef struct{
  float ax, ay, az;  // accelerometer data (m/s^2)
  float gx, gy, gz;  // gyroscope data (rad/s)
  float roll, pitch, yaw;  // Euler angles from Madgwick filter
}imu_t;



// Struct to hold the state for a finite state machine
class fsm
{
private:
  void readLineSensors();
  void readLaserRangingSensor();
  void readIMU();

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
  MPU6050 mpu;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
  /* data */


  fsm(/* args */);
  ~fsm();
  void setState(int new_state);
  void updateTisTes();
  void calibrateIMU();
  void calculateOdometry();
  void readSensors();
};

fsm::fsm(/* args */)
{
  this->setState(0);

  // Initialize peripherals pins
  // Line sensors
  pinMode(IR1_pin, INPUT);
  pinMode(IR2_pin, INPUT);
  pinMode(IR3_pin, INPUT);
  pinMode(IR4_pin, INPUT);
  pinMode(IR5_pin, INPUT);
  // Motor encoders
  pinMode(LEFT_MOTOR_ENCODER_A_pin, INPUT_PULLUP);
  pinMode(LEFT_MOTOR_ENCODER_B_pin, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_ENCODER_A_pin, INPUT_PULLUP);
  pinMode(RIGHT_MOTOR_ENCODER_B_pin, INPUT_PULLUP);
  // Motor PWM outputs
  pinMode(LEFT_MOTOR_D0_pin, OUTPUT);
  pinMode(LEFT_MOTOR_D1_pin, OUTPUT);
  pinMode(RIGHT_MOTOR_D2_pin, OUTPUT);
  pinMode(RIGHT_MOTOR_D3_pin, OUTPUT);
  // I2C
  Wire.begin();   // Ultrasound
  Wire1.begin();  // IMU

  laser_ranging_sensor.lox.begin();
  mpu.initialize(Wire1);  // Initialize MPU6050 on I2C1
}

fsm::~fsm()
{
}

void fsm::setState(int new_state){
  if (state != new_state) {  // if the state chnanged tis is reset
    state = new_state;
    tes = millis();
    tis = 0;
  }
}

void fsm::readLineSensors(){
  line_sensors.IR1 = digitalRead(IR1_pin);
  line_sensors.IR5 = digitalRead(IR5_pin);

  line_sensors.IR2 = analogRead(IR2_pin);
  line_sensors.IR3 = analogRead(IR3_pin);
  line_sensors.IR4 = analogRead(IR4_pin);


  uint32_t sum = line_sensors.IR2 + line_sensors.IR3 + line_sensors.IR4;
  if (sum > 0) {
    line_sensors.central_distance = (line_sensors.IR2 * 0.25 +
                                    line_sensors.IR3 * 0.5 +
                                    line_sensors.IR4 * 0.25) / 4095.0; // normalize for 12-bit ADC
  } else {
    line_sensors.central_distance = 0;
  }
}

void fsm::readLaserRangingSensor(){
  laser_ranging_sensor.lox.rangingTest(&laser_ranging_sensor.measure, false);
  if (laser_ranging_sensor.measure.RangeStatus != 4) {  // 4 = out of range
    laser_ranging_sensor.outOfRange=1;
  }else{
    laser_ranging_sensor.outOfRange=0;
    laser_ranging_sensor.distance=laser_ranging_sensor.measure.RangeMilliMeter;
  }
}

void fsm::readIMU(){
  int16_t ax, ay, az, gx, gy, gz;
  
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  // Convert to standard units (assuming ±2g for accel, ±250°/s for gyro)
  imu.ax = ax / 16384.0f;  // 16384 LSB/g for ±2g range
  imu.ay = ay / 16384.0f;
  imu.az = az / 16384.0f;
  
  imu.gx = gx / 131.0f;    // 131 LSB/(°/s) for ±250°/s range, convert to rad/s
  imu.gy = gy / 131.0f;
  imu.gz = gz / 131.0f;
}


void fsm::readSensors(){
  this->readLineSensors();
  this->readLaserRangingSensor();
  this->readIMU();
}

void fsm::updateTisTes(){
  uint32_t cur_time = millis();   // Just one call to millis()
  tis = cur_time - tes;
}

void fsm::calculateOdometry(){
  // Calculate distances traveled by each wheel
  // d1 = k * IMP1 (left wheel)
  // d2 = k * IMP2 (right wheel)
  odometry.d1 = CONVERSION_FACTOR_K * odometry.left_wheel_ticks;
  odometry.d2 = CONVERSION_FACTOR_K * odometry.right_wheel_ticks;
  
  // Calculate average distance traveled
  // d = (d1 + d2) / 2
  odometry.d = (odometry.d1 + odometry.d2) / 2.0f;
  
  // Calculate change in heading angle (delta_theta)
  // delta_theta = (d2 - d1) / WHEEL_DISTANCE__FACTOR_MM
  float delta_theta = (odometry.d2 - odometry.d1) / WHEEL_DISTANCE__FACTOR_MM;
  
  // Update position using kinematic equations
  // x = x + d*cos(theta + delta_theta/2)
  // y = y + d*sin(theta + delta_theta/2)
  // theta = theta + delta_theta
  
  float theta_mid = odometry.theta + delta_theta / 2.0f;
  odometry.x = odometry.x + odometry.d * cosf(theta_mid);
  odometry.y = odometry.y + odometry.d * sinf(theta_mid);
  odometry.theta = odometry.theta + delta_theta;
  
  // Reset encoder ticks for next iteration
  odometry.left_wheel_ticks = 0;
  odometry.right_wheel_ticks = 0;
}

fsm fsmLineFollower;

// meaningful names for the fsm1 states
enum {
  IDLE = 0,
  CALIBRATION_IMU = 1,
  LINE_FOLLOW = 2
};


uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t blink_period;

void setup() 
{
  fsmLineFollower.setState(0);


  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  blink_period = 1000 * 1.0/0.33; // In ms

  interval = 10;
}


void loop() 
{
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
      b = Serial.read();       
      if (b == '-') blink_period = 100 * blink_period / 80;  // Press '-' to decrease the frequency
      if (b == '+') blink_period = 80 * blink_period / 100;  // Press '+' to increase the frequency
    }

    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    uint32_t now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;

      // FSM processing - Calculate next state and perform actions
      switch(fsmLineFollower.state) {
        case IDLE:
          // Actions
          fsmLineFollower.motors.left_motor_speed = 0;
          fsmLineFollower.motors.right_motor_speed = 0;
          Serial.println("STATE: IDLE");
          
          // State transition
          if (fsmLineFollower.tis > 100) {
            fsmLineFollower.new_state = CALIBRATION_IMU;
          }
          break;

        case CALIBRATION_IMU:
          fsmLineFollower.readSensors();
          fsmLineFollower.motors.left_motor_speed = 0;
          fsmLineFollower.motors.right_motor_speed = 0;
          if (fsmLineFollower.tis < 1000) {
            Serial.println("STATE: CALIBRATION_IMU - Keep robot still");
          }
          
          // State transition
          if (fsmLineFollower.tis > 2000) {
            fsmLineFollower.new_state = LINE_FOLLOW;
          }
          break;

        case LINE_FOLLOW:

          fsmLineFollower.new_state = LINE_FOLLOW;
          break;

        default:
          fsmLineFollower.new_state = IDLE;
          break;
      }

      // Update tis and state
      fsmLineFollower.updateTisTes();
      fsmLineFollower.setState(fsmLineFollower.new_state);


      // Debug using the serial port
      /*
      Serial.print("S1: ");
      Serial.print(S1);

      Serial.print(" S2: ");
      Serial.print(S2);

      Serial.print(" fsm1.state: ");
      Serial.print(fsm1.state);

      Serial.print(" LED_1: ");
      Serial.print(LED_1);

      Serial.print(" LED_2: ");
      Serial.print(LED_2);

      Serial.print(" blink: ");
      Serial.print(blink_period);

      Serial.print(" loop: ");
      Serial.print(micros() - loop_micros);
      Serial.println();
      */
    }
    
}
