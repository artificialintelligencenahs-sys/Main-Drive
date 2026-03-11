#include "robot_config.h"
#include "pros/adi.hpp"
#include "pros/motors.h"


// Vertical Tracking Wheel
pros::Rotation rotation_sensor(-12);

pros::Imu imu(11);

pros::Optical optical_sensor(20);

pros::Distance back_sensor_left(19);
pros::Distance back_sensor_right(17);
pros::Distance left_sensor(18);
pros::Distance right_sensor(16);

const double back_sensor_left_offset  = 4.875;
const double back_sensor_right_offset = 4.875;
const double back_sensor_spacing      = 9.125;
const double left_sensor_offset       = 5.8125;
const double right_sensor_offset      = 5.8125;
const double field_half_size          = 72.0;

pros::MotorGroup left_motors({1, -3, -4}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-2, 5, 6}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450rpm (im pretty sure)
                              5 // horizontal drift (omni + traction wheel mix)
);

// tracking wheel configuration
lemlib::TrackingWheel vertical_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_275, .375);
// vertical wheel, 2.75" diameter, -.25" offset from tracking center

// odometry sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // no second vertical wheel
                            nullptr, // no horizontal tracking wheel
                            nullptr, // no second horizontal wheel
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(4.3,   // kP
                                              0,   // kI
                                              9,   // kD
                                              3,   // anti windup
                                              1,   // small error range
                                              100, // small error range timeout
                                              3,   // large error range
                                              500, // large error range timeout
                                              0  // slew - START HIGH
);


// angular PID controller - TUNED
lemlib::ControllerSettings angular_controller(.86,//kP
                                              0,   // kI
                                              0.2, //kD
                                              3,   // anti windup - ENABLE
                                              1,   // small error range, in degrees - ENABLE
                                              100, // small error range timeout - ENABLE
                                              3,   // large error range, in degrees - ENABLE
                                              500, // large error range timeout - ENABLE
                                              0    // slew
);

// create the chassis (no LemLib curve — we apply our own in opcontrol)
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);

// --------------------- Motors ---------------------

pros::Motor Intake(INTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Motor Outtake(OUTTAKE_PORT, pros::MotorGears::blue, pros::MotorUnits::degrees);

// --------------------- Sensors ---------------------
pros::adi::DigitalOut Descore('A');
pros::adi::DigitalOut Unloader('B');
pros::adi::DigitalOut MidScoring('C');
pros::adi::DigitalOut MidDescore('D');

// --------------------- Controller ---------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);



void initializeRobot() {
    // Non-blocking calibration - fixes "Run" mode hang
    chassis.calibrate(false);
    
    // Set brake modes
    Outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);        // Holds position, prevents backdriving    
    Outtake.set_reversed(false);
    Intake.set_reversed(true);
    MidScoring.set_value(false);
    MidDescore.set_value(false);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


}