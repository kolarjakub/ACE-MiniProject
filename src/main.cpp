
#include <Arduino.h>
#include "fsm.h"

uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t blink_period;


// Instantiate the global FSM instance
fsm fsmLineFollower;

void setup() 
{
  fsmLineFollower.setState(0);
  fsmLineFollower.setVelocityReferences(0,0);
  fsmLineFollower.writeOutputs();
  fsmLineFollower.readSensors();




  // Start the serial port with 115200 baudrate
  Serial.begin(115200);

  blink_period = 1000 * 1.0/0.33; // In ms

  interval = 40;
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

      fsmLineFollower.calculateOdometry();
      fsmLineFollower.readSensors();

      // FSM processing - Calculate next state and perform actions
      switch(fsmLineFollower.state) {
        case IDLE:
          // Actions
          //fsmLineFollower.motors.left_motor_speed = 0;
          //fsmLineFollower.motors.right_motor_speed = 0;
          fsmLineFollower.setVelocityReferences(0,0);
          Serial.println("STATE: IDLE");
          
          // State transition
          if (fsmLineFollower.tis > 100) {
            fsmLineFollower.new_state = CALIBRATION_IMU;
          }
          break;

        case CALIBRATION_IMU:
          //fsmLineFollower.motors.left_motor_speed = 0;
          //fsmLineFollower.motors.right_motor_speed = 0;
          fsmLineFollower.setVelocityReferences(0,0);
          if (fsmLineFollower.tis < 1000) {
            Serial.println("STATE: CALIBRATION_IMU - Keep robot still");
          }
          
          // State transition
          if (fsmLineFollower.tis > 2000) {
            fsmLineFollower.new_state = LINE_FOLLOW;
          }
          break;

        case LINE_FOLLOW:
          // IDEA
          fsmLineFollower.setVelocityReferences(3,0);
          //fsmLineFollower.new_state = LINE_FOLLOW;
          break;

        default:
          fsmLineFollower.new_state = IDLE;
          break;
      }

      fsmLineFollower.writeOutputs();

      // Update tis and state
      fsmLineFollower.updateTisTes();
      fsmLineFollower.setState(fsmLineFollower.new_state);
    }
 
    Serial.print(" loop: ");
    Serial.print(micros() - loop_micros);
    Serial.println();

}