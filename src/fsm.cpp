#include "fsm.h"
#include <math.h>

// Volatile counters updated from ISRs
static volatile int32_t encoder_left_count = 0;
static volatile int32_t encoder_right_count = 0;

// Forward declarations for ISRs
static void leftEncoderA_isr();
static void leftEncoderB_isr();
static void rightEncoderA_isr();
static void rightEncoderB_isr();

// Constructor
fsm::fsm()
{
    this->setState(IDLE);

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

    // Attach encoder interrupts (quadrature decoding)
    int leftAint = digitalPinToInterrupt(LEFT_MOTOR_ENCODER_A_pin);
    int leftBint = digitalPinToInterrupt(LEFT_MOTOR_ENCODER_B_pin);
    int rightAint = digitalPinToInterrupt(RIGHT_MOTOR_ENCODER_A_pin);
    int rightBint = digitalPinToInterrupt(RIGHT_MOTOR_ENCODER_B_pin);

    if (leftAint != NOT_AN_INTERRUPT) attachInterrupt(leftAint, leftEncoderA_isr, CHANGE);
    if (leftBint != NOT_AN_INTERRUPT) attachInterrupt(leftBint, leftEncoderB_isr, CHANGE);
    if (rightAint != NOT_AN_INTERRUPT) attachInterrupt(rightAint, rightEncoderA_isr, CHANGE);
    if (rightBint != NOT_AN_INTERRUPT) attachInterrupt(rightBint, rightEncoderB_isr, CHANGE);

    // Motor PWM outputs
    pinMode(LEFT_MOTOR_D0_pin, OUTPUT);
    pinMode(LEFT_MOTOR_D1_pin, OUTPUT);
    pinMode(RIGHT_MOTOR_D2_pin, OUTPUT);
    pinMode(RIGHT_MOTOR_D3_pin, OUTPUT);

    // IMU MPU6500
    pinMode(IMU_SDA_pin, INPUT_PULLUP);
    pinMode(IMU_SCL_pin, INPUT_PULLUP);
    Wire1.setSDA(IMU_SDA_pin);
    Wire1.setSCL(IMU_SCL_pin);
    Wire1.begin();
    MPU6500Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    while(!mpu.setup(0x68, setting)) {
        Serial.println("MPU connection failed.");
        while (1);
    }Serial.println("MPU initialized.");

    // Laser Ranging Sensor
    pinMode(IMU_SDA_pin, INPUT_PULLUP);
    pinMode(IMU_SCL_pin, INPUT_PULLUP);
    Wire.setSDA(LASER_RANGING_SDA_pin);
    Wire.setSCL(LASER_RANGING_SCL_pin);
    Wire.begin();   // Ultrasound
    if (!laser_ranging_sensor.lox.begin()) {
        Serial.println("Failed to initialize VL53L0X! Check your wiring.");
        while (1);
    }
    Serial.println("VL53L0X initialized.");
}

fsm::~fsm()
{
}

void fsm::setState(int new_state){
  if (state != new_state) {  // if the state chnaged tis is reset
    state = new_state;
    tes = millis();
    tis = 0;
  }
}

void fsm::readEncoders() {
    // Snapshot and clear the ISR-updated counters into the odometry structure.
    // This function should be called from the main loop at a regular interval
    // (e.g. every ~40 ms) to obtain the number of encoder ticks since the
    // last call.
    noInterrupts(); // disable interrupts while we copy and clear
    int32_t l = encoder_left_count;
    int32_t r = encoder_right_count;
    encoder_left_count = 0;
    encoder_right_count = 0;
    interrupts(); // re-enable interrupts

    // store into the odometry fields (signed ticks since last read)
    odometry.left_wheel_ticks = l;
    odometry.right_wheel_ticks = r;

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
      // Read, if ready, the MPU - from previous project
    if (mpu.update()) {
      imu.last_cycle_time = imu.cycle_time;
      imu.cycle_time = micros();

      imu.last_cycle_time = imu.cycle_time;
      imu.cycle_time = micros();

      imu.w.x = mpu.getGyroX();
      imu.w.y = mpu.getGyroY();
      imu.w.z = mpu.getGyroZ();

      imu.a.x = mpu.getAccX();
      imu.a.y = mpu.getAccY();
      imu.a.z = mpu.getAccZ();
    }

/*
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
  */
}


void fsm::readSensors(){
    this->readEncoders();
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
  odometry.dl = CONVERSION_FACTOR_K * odometry.left_wheel_ticks;
  odometry.dr = CONVERSION_FACTOR_K * odometry.right_wheel_ticks;
  
  // Calculate average distance traveled
  // d = (d1 + d2) / 2
  odometry.d = (odometry.dl + odometry.dr) / 2.0f;
  
  // Calculate change in heading angle (delta_theta)
  // delta_theta = (d2 - d1) / WHEEL_DISTANCE__FACTOR_MM
  float delta_theta = (odometry.dr - odometry.dl) / WHEEL_DISTANCE_M;
  
  // Update position using kinematic equations
  // x = x + d*cos(theta + delta_theta/2)
  // y = y + d*sin(theta + delta_theta/2)
  // theta = theta + delta_theta
  float theta_mid = odometry.theta + delta_theta / 2.0f;
  odometry.x = odometry.x + odometry.d * cosf(theta_mid);
  odometry.y = odometry.y + odometry.d * sinf(theta_mid);
  odometry.theta = odometry.theta + delta_theta;



    // Estimate wheels speed using the encoders
  odometry.wl = odometry.left_wheel_ticks * TWO_PI / (2.0 * 1920.0 * DT);
  odometry.wr = odometry.right_wheel_ticks * TWO_PI / (2.0 * 1920.0 * DT);

  odometry.vl = odometry.wl * WHEEL_RADIUS_M;
  odometry.vr = odometry.wr * WHEEL_RADIUS_M;

  // Estimate robot speed
  odometry.v = (odometry.vl + odometry.vr) / 2.0;
  odometry.w = (odometry.vl - odometry.vr) / WHEEL_DISTANCE_M;

  // Reset encoder ticks for next iteration
  odometry.left_wheel_ticks = 0;
  odometry.right_wheel_ticks = 0;
}


void fsm::updateVelocityPID()
{
    // Calculate PID output
    motors.left_motor_speed  = velocity_pid_controller.left.calc(odometry.wl_req, odometry.wl);
    motors.right_motor_speed = velocity_pid_controller.right.calc(odometry.wr_req, odometry.wr);
}


void fsm::setVelocityReferences(float v_req, float w_req){
    // Convert (v_req, w_req) to (wl_req, wr_req)
    odometry.wl_req = (v_req - (w_req * WHEEL_DISTANCE_M / 2.0f)) / WHEEL_RADIUS_M;
    odometry.wr_req = (v_req + (w_req * WHEEL_DISTANCE_M / 2.0f)) / WHEEL_RADIUS_M;

    odometry.w_req = w_req;
    odometry.v_req = v_req;
}


void fsm::writeOutputs(){
    // Left motor control
    this->updateVelocityPID();

    if (motors.left_motor_speed >= 0) {
        analogWrite(LEFT_MOTOR_D0_pin, motors.left_motor_speed);
        analogWrite(LEFT_MOTOR_D1_pin, 0);
    } else {
        analogWrite(LEFT_MOTOR_D0_pin, 0);
        analogWrite(LEFT_MOTOR_D1_pin, -motors.left_motor_speed);
    }
    
    // Right motor control
    if (motors.right_motor_speed >= 0) {
        analogWrite(RIGHT_MOTOR_D2_pin, motors.right_motor_speed);
        analogWrite(RIGHT_MOTOR_D3_pin, 0);
    } else {
        analogWrite(RIGHT_MOTOR_D2_pin, 0);
        analogWrite(RIGHT_MOTOR_D3_pin, -motors.right_motor_speed);
    }
}

// --- Encoder ISR implementations (quadrature decoding) ---
static void leftEncoderA_isr() {
  uint8_t a = digitalRead(LEFT_MOTOR_ENCODER_A_pin);
  uint8_t b = digitalRead(LEFT_MOTOR_ENCODER_B_pin);
  if (a == b) {
    encoder_left_count++;
  } else {
    encoder_left_count--;
  }
}

static void leftEncoderB_isr() {
  uint8_t a = digitalRead(LEFT_MOTOR_ENCODER_A_pin);
  uint8_t b = digitalRead(LEFT_MOTOR_ENCODER_B_pin);
  // when B changes, if B != A direction is +, else -
  if (b != a) {
    encoder_left_count++;
  } else {
    encoder_left_count--;
  }
}

static void rightEncoderA_isr() {
  uint8_t a = digitalRead(RIGHT_MOTOR_ENCODER_A_pin);
  uint8_t b = digitalRead(RIGHT_MOTOR_ENCODER_B_pin);
  if (a == b) {
    encoder_right_count++;
  } else {
    encoder_right_count--;
  }
}

static void rightEncoderB_isr() {
  uint8_t a = digitalRead(RIGHT_MOTOR_ENCODER_A_pin);
  uint8_t b = digitalRead(RIGHT_MOTOR_ENCODER_B_pin);
  if (b != a) {
    encoder_right_count++;
  } else {
    encoder_right_count--;
  }
}
