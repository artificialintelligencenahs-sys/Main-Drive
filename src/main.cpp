#include "main.h"
#include "autonomous.h"
#include "helper_functions.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/pneumatics.h"
#include <string>


void initialize() { initializeRobot(); }

void disabled() {}

void competition_initialize() {}

// Custom drive curve: clean deadband + power curve, no LemLib quirks
// deadband:  stick values at or below this output exactly 0
// minOutput: motor value the instant the stick crosses the deadband
// gain:      1.0 = linear, 1.5 = gentle curve, 2.0+ = more low-speed precision
static float driveCurve(float input, float deadband = 5, float minOutput = 15, float gain = 1.3) {
    if (std::fabs(input) <= deadband) return 0;
    float sign   = input > 0 ? 1.0f : -1.0f;
    float x      = (std::fabs(input) - deadband) / (127.0f - deadband);
    float curved = std::pow(x, gain);
    return sign * (minOutput + (127.0f - minOutput) * curved);
}
void printPoseTask(void*) {
  while (true) {
    lemlib::Pose pose = chassis.getPose();
    master.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
    pros::delay(200); // controller screen can't update faster than ~50-200ms
    master.print(1, 0, "H:%.1f        ", pose.theta);
    pros::delay(200);
  }
}

void autonomous() {
  int auton_selected = 2;
  switch(auton_selected) {
    case 1:
      swp();
      break;  
    case 2:
      skills_auton();
      break;
    case 3:
      leftAuton();
      break;  
    case 4:
      leftAuton_descore();
      break;
    case 5:
      rightAuton();
      break;
    case 6:
      score();
      break;
    case 7:
      park();
      break;
    case 8:
      tuning();
      break;
    case 9:
      distanceresettest();
      break;
      

  }
}



void opcontrol() {
  IntakeControl intake;
  OuttakeControl outtake;
  PneumaticControl pneumatics;

  // Initialize descore piston to extended position
  Descore.set_value(true);



  // Tracking for warnings (don't spam alerts)
  uint32_t lastTempCheck = 0;
  uint32_t lastBatteryCheck = 0;
  bool lowBatteryWarned = false;

  while (true) {

    // Display intake current draw on brain screen
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::fill_rect(0, 0, 480, 40);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(
        pros::E_TEXT_LARGE, 10, 10, "Intake mA: %d",
        Intake.get_current_draw());

    // Tank Drive with our custom curve (LemLib curve disabled)
    int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank((int)driveCurve(leftY), (int)driveCurve(rightY), true);

    // Update subsystems
    outtake.update(intake);
    intake.update(outtake);
    pneumatics.update();


    // === MOTOR TEMPERATURE MONITORING (every 2 seconds) ===
    if (pros::millis() - lastTempCheck > 2000) {
      lastTempCheck = pros::millis();

      // Check all motor temperatures (overheat starts at 55°C)
      double maxTemp = 0;
      std::string hotMotor = "";

      // Check drive motors (get_temperature returns vector for groups)
      auto leftTemps = left_motors.get_temperature_all();
      auto rightTemps = right_motors.get_temperature_all();

      for (double temp : leftTemps) {
        if (temp > maxTemp) {
          maxTemp = temp;
          hotMotor = "L-Drive";
        }
      }
      for (double temp : rightTemps) {
        if (temp > maxTemp) {
          maxTemp = temp;
          hotMotor = "R-Drive";
        }
      }
      if (Intake.get_temperature() > maxTemp) {
        maxTemp = Intake.get_temperature();
        hotMotor = "Intake";
      }
      if (Outtake.get_temperature() > maxTemp) {
        maxTemp = Outtake.get_temperature();
        hotMotor = "Outtake";
      }

      // Warn at 50°C (before power reduction kicks in at 55°C)
      if (maxTemp >= 50) {
        master.print(0, 0, "HOT: %s %.0fC   ", hotMotor.c_str(), maxTemp);
      }
    }

    // === LOW BATTERY WARNING (10%) ===
    if (pros::millis() - lastBatteryCheck > 5000) {
      lastBatteryCheck = pros::millis();

      int batteryLevel = pros::battery::get_capacity();
      if (batteryLevel <= 10 && !lowBatteryWarned) {
        master.rumble("---"); // Long rumble pattern
        master.print(1, 0, "LOW BATTERY: %d%%", batteryLevel);
        lowBatteryWarned = true;
      }
    }

    pros::delay(20);
  }
}
