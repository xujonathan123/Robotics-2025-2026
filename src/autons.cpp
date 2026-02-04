#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127.
// Conservative anti-tip speeds for a light robot.
const int DRIVE_SPEED = 58;  // full speed is 127
const int TURN_SPEED = 40;
const int SWING_SPEED = 65;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(8.0, 0.0, 35.0);         // IMU heading hold during drives
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(6_deg, 35);
  chassis.slew_drive_constants_set(6_in, 35);
  chassis.slew_swing_constants_set(6_in, 40);

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there 
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.drive_imu_reset();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  chassis.drive_imu_reset();
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

void left_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 35, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn left another ~70 degrees (to -160 total)
  chassis.pid_turn_set(-160_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 5) Back up 30 in
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(1000);
  intake.move(0);
  intake_stage2.move(0);
}

void right_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 35, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn right 90 degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn right another ~70 degrees (to 160 total)
  chassis.pid_turn_set(160_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 5) Back up 30 in
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(1000);
  intake.move(0);
  intake_stage2.move(0);
}

