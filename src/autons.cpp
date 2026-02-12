#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127.
// Conservative anti-tip speeds for a light robot.
const int DRIVE_SPEED = 75;  // full speed is 127
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

namespace {
void stop_all_motors() {
  chassis.drive_set(0, 0);
  intake.move(0);
  intake_stage2.move(0);
}
}  // namespace

void left_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 40, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn left another ~50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 5) Back up 30 in
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void right_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 40, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn right another ~50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);


  // 5) Back up 30 in
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void left_start_matchload() {
  chassis.drive_imu_reset();

  // 0.5) start match with matchloader raised
  matchload_set(true);
  pros::delay(250);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 35, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn left another ~50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 4.5) lower match load
  matchload_set(false);
  pros::delay(160);

  // 5) Drive forward 20 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(20_in, 35, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 6) Back up 30 in
  chassis.pid_drive_set(-50_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 7) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void temp_skills() {
  intake.move(127);
  intake_stage2.move(-127);
  pros::delay(2000);
  intake_stage2.move(0);
  intake.move(0);
  stop_all_motors();
}

void right_start_skills() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 43 in
  chassis.pid_drive_set(43_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn right another ~50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 5) Back up 25 in
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 7) reset imu for way back
  chassis.drive_imu_reset();

  // 8) Drive forward 32 inches
  chassis.pid_drive_set(32_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 9) drive backward 40 inches
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 10) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 11) IMU turn right 90 degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 12) Drive forward 120 inches
  chassis.pid_drive_set(124_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 13) IMU turn left 90 degrees (return to step 7 orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 14) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 15) Drive backward 35 inches
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 16) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 17) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);
  
  // 18) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 19) Drive forward 60 inches
  chassis.pid_drive_set(62_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 20) IMU turn left 90 degrees (return to step 7 orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 21) Drive 40 inches into parking spot and clear park zone 
  intake.move(127);
  intake_stage2.move(127);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void left_start_skills() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 43 in
  chassis.pid_drive_set(43_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn left another ~50 degrees (to 170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 5) Back up 25 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 7) reset imu for way back
  chassis.drive_imu_reset();

  // 8) Drive forward 32 inches
  chassis.pid_drive_set(32_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 9) drive backward 40 inches
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 10) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 11) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 12) Drive forward 120 inches
  chassis.pid_drive_set(121_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();  
  pros::delay(160);

  // 13) IMU turn right 90 degrees (return to step 7 orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 14) Drive forward 15 inches
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 15) Drive backward 35 inches
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 16) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 17) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);
  
  // 18) IMU turn right 90 degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 19) Drive forward 60 inches
  chassis.pid_drive_set(61_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 20) IMU turn left 90 degrees (return to step 7 orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 21) Drive 40 inches into parking spot and clear park zone 
  intake.move(127);
  intake_stage2.move(127);
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}
