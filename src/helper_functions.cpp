

// ============================================================================

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "pros/optical.hpp"
#include "helper_functions.h"
#include "robot_config.h"
#include <cmath>





const double back_sensor_left_offset  = 4.875;
const double back_sensor_right_offset = 4.875;
const double back_sensor_spacing      = 10.5;
const double left_sensor_offset       = 5.8125;
const double right_sensor_offset      = 5.8125;
const double field_half_size          = 72.0;


void resetPositionAndHeadingBack(pros::Distance& back_left, pros::Distance& back_right,
                                  double sensor_spacing,
                                  double left_offset,   double right_offset,
                                  double field_half) {

    double d_left  = (back_left.get()+19)  / 25.4; // mm to inches
    double d_right = back_right.get() / 25.4;

    // Validate readings
    if (d_left < 0 || d_left > 200 || d_right < 0 || d_right > 200) {
        printf("Invalid back sensor readings: L=%.1f R=%.1f\n", d_left, d_right);
        return;
    }

    // calculate angle to wall from the two sensor readings
    // positive angle = robot is rotated clockwise from perpendicular
    double angle_to_wall_rad = atan2(d_right - d_left, sensor_spacing);
    double angle_to_wall_deg = angle_to_wall_rad * 180.0 / M_PI;

    // calculate corrected perpendicular distance
    double avg_offset   = (left_offset + right_offset) / 2.0;
    double avg_reading  = (d_left + d_right) / 2.0;
    double corrected_distance = avg_reading * cos(angle_to_wall_rad) + avg_offset;

    // determine which wall the BACK of the robot is facing
    // back of robot = heading + 180°
    lemlib::Pose pose = chassis.getPose();
    double back_heading_deg = pose.theta + 180.0;
    int headingDeg = ((int)back_heading_deg % 360 + 360) % 360;

    bool   resettingX = false;
    double wallSign   = 1.0;
    double expected_perpendicular_heading = 0.0;

    if (headingDeg >= 315 || headingDeg <= 45) {
        // back faces top wall → reset Y (positive side)
        resettingX = false;
        wallSign   = 1.0;
        expected_perpendicular_heading = 180.0;
    }
    else if (headingDeg > 45 && headingDeg <= 135) {
        // back faces right wall → reset X (positive side)
        resettingX = true;
        wallSign   = 1.0;
        expected_perpendicular_heading = 270.0;
    }
    else if (headingDeg > 135 && headingDeg <= 225) {
        // back faces bottom wall → reset Y (negative side)
        resettingX = false;
        wallSign   = -1.0;
        expected_perpendicular_heading = 0.0;
    }
    else {
        // back faces left wall → reset X (negative side)
        resettingX = true;
        wallSign   = -1.0;
        expected_perpendicular_heading = 90.0;
    }

    // calculate corrected position
    double actualPos = wallSign * (field_half - corrected_distance);

    // calculate corrected heading
    // subtracting the angle correctly converts to global heading
    double corrected_heading = expected_perpendicular_heading - angle_to_wall_deg;

    // normalize heading to 0-360
    corrected_heading = fmod(corrected_heading + 360, 360);

    // apply corrected pose - only update the relevant axis, keep the other
    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, corrected_heading);

    printf("Reset: pos=%.1f hdg=%.1f (wall_angle=%.1f)\n",
           actualPos, corrected_heading, angle_to_wall_deg);
}


void resetPositionLeft(pros::Distance& sensor, double sensor_offset,
                       double field_half) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid left sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Left sensor direction = robot heading + 270° (pointing left)
    double sensor_heading_deg = pose.theta + 270.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction: find how far off perpendicular we are from the nearest wall
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    // Correct the reading for the angle
    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}


void resetPositionRight(pros::Distance& sensor, double sensor_offset,
                        double field_half) {

    double sensorReading = sensor.get() / 25.4;

    if (sensorReading < 0 || sensorReading > 200) {
        printf("Invalid right sensor reading: %.2f\n", sensorReading);
        return;
    }

    lemlib::Pose pose = chassis.getPose();

    // Right sensor direction = robot heading + 90°
    double sensor_heading_deg = pose.theta + 90.0;
    int headingDeg = ((int)sensor_heading_deg % 360 + 360) % 360;

    // Trig correction
    double nearest_perpendicular = round(sensor_heading_deg / 90.0) * 90.0;
    double angle_off_deg = sensor_heading_deg - nearest_perpendicular;
    double angle_off_rad = angle_off_deg * M_PI / 180.0;

    double corrected_distance = sensorReading * cos(angle_off_rad) + sensor_offset;

    // Determine which wall
    bool   resettingX = false;
    double wallSign   = 1.0;

    if      (headingDeg >= 315 || headingDeg <= 45)  { resettingX = false; wallSign =  1.0; } // Top wall
    else if (headingDeg > 45  && headingDeg <= 135)  { resettingX = true;  wallSign =  1.0; } // Right wall
    else if (headingDeg > 135 && headingDeg <= 225)  { resettingX = false; wallSign = -1.0; } // Bottom wall
    else                                              { resettingX = true;  wallSign = -1.0; } // Left wall

    double actualPos = wallSign * (field_half - corrected_distance);

    double new_x = resettingX ? actualPos : pose.x;
    double new_y = resettingX ? pose.y    : actualPos;
    chassis.setPose(new_x, new_y, pose.theta);
}


// driveUntilDistance
// drives the robot until a distance sensor reads below a threshold, then stops.


void driveUntilDistance(pros::Distance& sensor, double threshold_in,
                        int speed, bool forwards, int timeout_ms) {

    int direction = forwards ? 1 : -1;
    int elapsed   = 0;

    chassis.tank(direction * speed, direction * speed, true);

    while (elapsed < timeout_ms) {
        // Only check distance if the sensor confirms an object is in range
        if (sensor.get() != PROS_ERR) {
            double reading = sensor.get() / 25.4;
            if (reading > 0 && reading <= threshold_in) {
                break;
            }
        }

        pros::delay(10);
        elapsed += 10;
    }

    // Stop with hold brake to prevent drift after stopping
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.tank(0, 0, true);
}

const double normal_motor_velocity = 625; 
const double jam_threshold = 20;
const int AVG_SIZE = 5;

void score() {
    Intake.move(-127); // move intake and outtake
    Outtake.move(127);

    pros::delay(500);

    double velocityBuffer[AVG_SIZE] = {0};
    int bufferIndex = 0;

    auto getAvgVelocity = [&]() {
        double sum = 0;
        for (int i = 0; i < AVG_SIZE; i++) sum += velocityBuffer[i]; // list of 5 last velocity readings
        return sum / AVG_SIZE;
    };

    uint32_t startTime = pros::millis(); // Timeout start

    while (true) {
        velocityBuffer[bufferIndex] = std::abs(Intake.get_actual_velocity()); // keeps iterating through list, replacing with last velocity
        bufferIndex = (bufferIndex + 1) % AVG_SIZE;
        double velocity = getAvgVelocity();

        // Unjam if velocity is hella low
        if (velocity <= jam_threshold) {
            Intake.move(96);
            Outtake.move(-96);
            pros::delay(150);

            Intake.move(-127);
            Outtake.move(127);
            pros::delay(500); // let it spin back up before checking again
        }

        // All balls scored once velocity returns to normal
        if (velocity >= normal_motor_velocity) {
            pros::delay(1000);
            break;
        }

        // Timeout — if scoring hasn't completed in 5 seconds, give up
        if (pros::millis() - startTime >= 5000) {
            break;
        }

        pros::delay(20);
    }

    Intake.move(0);
    Outtake.move(0);
}



void park() {
    chassis.cancelMotion();
    optical_sensor.set_led_pwm(100);
    Intake.move(-127);

    while (true) {
        // move forward
        chassis.arcade(67, 0);
        uint32_t startTime = pros::millis();
        bool redDetected = false;
        // keep moving until red is detected or 5 seconds pass
        while (true) {
            double hue = optical_sensor.get_hue();
            if (hue > 350 || hue < 20) {
                redDetected = true;
                break;
            }
            // if 5 seconds have passed without detecting red
            if (pros::millis() - startTime >= 5000) {
                break;
            }
            pros::delay(10);
        }
        if (redDetected) {
            break; // exit outer loop, red present
        }
        // red not found - reverse for 1 second and try again
        chassis.arcade(-67, 0);
        pros::delay(500);
        chassis.arcade(0, 0);
        pros::delay(100); // brief pause before retrying
    }
    // Stop the bot
    chassis.arcade(0, 0);
}


void shakeBot(int durationMs) {
  const int SHAKE_POWER = 35;     // motor power for each shake (tune if needed)
  const int SHAKE_INTERVAL = 100; // ms per half-cycle (left or right)

  uint32_t startTime = pros::millis();

  while ((pros::millis() - startTime) < (uint32_t)durationMs) {
    // left motors backward right motors forward
    left_motors.move(-SHAKE_POWER);
    right_motors.move(SHAKE_POWER);
    pros::delay(SHAKE_INTERVAL);

    // left motors forward, right motors backward
    left_motors.move(SHAKE_POWER);
    right_motors.move(-SHAKE_POWER);
    pros::delay(SHAKE_INTERVAL);
  }

  // stop
  left_motors.brake();
  right_motors.brake();
}

void score_ms(int ms) {
    Intake.move(-127);
    Outtake.move(127);
    pros::delay(ms);
    Intake.move(0);
    Outtake.move(0);
}