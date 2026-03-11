#include "autonomous.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "helper_functions.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <ctime>

const double normal_motor_velocity = 625; 
const double jam_threshold = 20;
const int AVG_SIZE = 5;

void tuning() {
    chassis.setPose(0,0,0);
    chassis.turnToHeading(90, 9999);
}

void swp() {
    chassis.setPose(0,0,90);
    chassis.moveToPoint(30, 0, 750);
    chassis.turnToHeading(180, 500,{},false);
    Unloader.set_value(true);
    Intake.move(-127);
    chassis.moveToPoint(30, -100, 500,{.maxSpeed=50},false);
   // left_motors.move(30);
    //right_motors.move(30);
    pros::delay(600);
    //left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(30, 30, 1000,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
    pros::delay(400);
    Outtake.move(0);
    chassis.moveToPoint(30, 12, 800,{.minSpeed=50,.earlyExitRange=1});
    chassis.turnToPoint(7, 25, 800);
    chassis.moveToPoint(7, 25, 1000);
    chassis.turnToHeading(-80, 200,{.minSpeed=40});
    chassis.turnToPoint(-40, 25, 500);
    chassis.moveToPoint(-40, 25, 2550,{.minSpeed=50});
    chassis.turnToHeading(225,700);
    chassis.moveToPoint(-31.5, 37.5, 1250,{.forwards=false},false);
    MidScoring.set_value(true);
    Intake.move(-127);
    Outtake.move(-127);
    pros::delay(750);
    Outtake.move(0);
    chassis.moveToPoint(-68, 10, 1750);
    chassis.turnToHeading(180, 500,{},false);
    Unloader.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-68, -100, 500,{.maxSpeed=70},false);
  //  left_motors.move(30);
    //right_motors.move(30);
    pros::delay(600);
   // left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(-67, 30, 750,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
}

void distanceresettest(){
    chassis.setPose(48,80,0);
    resetPositionAndHeadingBack(back_sensor_left, back_sensor_right);
    resetPositionRight(right_sensor);
}


void skills_auton() { 
    chassis.setPose(0,0,270);
    chassis.moveToPoint(-31, 0, 750); //move to unloader
    chassis.turnToHeading(180, 500,{},false);
    Unloader.set_value(true);
    Intake.move(-127);
    pros::delay(200);
    chassis.moveToPoint(-31, -100, 500,{.maxSpeed=60},false); //unload 
   // left_motors.move(30);
    //right_motors.move(30);
    pros::delay(1250);
  //  left_motors.move(0);
   // right_motors.move(0);
    chassis.moveToPoint(-30, -8, 500,{.forwards=false,},false); //back a lil
    chassis.moveToPoint(-32, -100, 500,{.maxSpeed=60},false);
    chassis.moveToPoint(-30, 0, 1000,{.forwards=false,},false);
    Unloader.set_value(false);
    chassis.turnToHeading(135, 750);
    chassis.moveToPoint(-41, 15, 1000,{.forwards=false});
    chassis.turnToHeading(180, 750,{.minSpeed=20},false);
    resetPositionRight(right_sensor);
    chassis.moveToPoint(-62, 95, 1750,{.forwards=false,.maxSpeed=70});
    chassis.turnToHeading(90, 750,{},false); 

    resetPositionAndHeadingBack(back_sensor_left, back_sensor_right, 10.5);
    resetPositionLeft(left_sensor);
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(-50, pose.y, 1000); // move to goal plane
    chassis.turnToHeading(0, 750);
    chassis.moveToPoint(-50, 23, 1000,{.forwards=false,.maxSpeed=60},false); // go to long goal
    score();
    Intake.move(-127);
    Unloader.set_value(true);
    chassis.turnToPoint(-50, 100, 300); // unload
    chassis.moveToPoint(-50, 100, 850,{.maxSpeed=60},false);
    //left_motors.move(30);
    //right_motors.move(30);
    pros::delay(1500);
    //left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(-50, 50, 500,{.forwards=false,},false); // back a lil
    chassis.moveToPoint(-50, 100, 500,{.maxSpeed=60},false);
    chassis.moveToPoint(-50, 0, 1250,{.forwards=false,.maxSpeed=60},false);
    score(); // score again
    Unloader.set_value(false);
    chassis.moveToPoint(-49, 40, 1000);
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(60, 40, 2500,{.forwards=false,.maxSpeed=70},false); // move to unloader 3
    pros::delay(200);

    resetPositionAndHeadingBack(back_sensor_left, back_sensor_right);
    resetPositionRight(right_sensor);
    lemlib::Pose pose2 = chassis.getPose();
    chassis.moveToPoint(49, pose2.y, 1000,{.minSpeed=15}); // go to unloader
    chassis.turnToHeading(0, 750,{},false);
    Unloader.set_value(true);
    Intake.move(-127);
    pros::delay(200);
    chassis.moveToPoint(49, 100, 500,{.maxSpeed=60},false); //unload
    //left_motors.move(30);
    //right_motors.move(30);
    pros::delay(1350);
    //left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(49, 50, 500,{.forwards=false,},false); // back a lil
    chassis.moveToPoint(49, 100, 500,{.maxSpeed=60},false);
    chassis.moveToPoint(49,50, 1000,{.forwards=false,},false);
    Unloader.set_value(false);
    chassis.turnToHeading(315, 750);
    chassis.moveToPoint(67, 35, 1000, {.forwards=false});
    chassis.turnToHeading(0, 750,{.minSpeed=20},false);
    resetPositionRight(right_sensor);
    chassis.moveToPoint(61.5, -45, 1750,{.forwards=false,.maxSpeed=70});
    chassis.turnToHeading(270, 750,{},false);

    resetPositionAndHeadingBack(back_sensor_left, back_sensor_right);
    resetPositionLeft(left_sensor);
    lemlib::Pose pose3 = chassis.getPose();
    chassis.moveToPoint(50, pose3.y, 1000); // move to goal plane
    chassis.turnToHeading(180,750);
    chassis.moveToPoint(51, 80, 1000,{.forwards=false,.maxSpeed=60},false); // go to goal
    score();
    Intake.move(-127);
    Unloader.set_value(true);
    chassis.turnToPoint(49, -100, 300); // unload
    chassis.moveToPoint(49, -100, 1000,{.maxSpeed=60},false);
  //  left_motors.move(30);
    //right_motors.move(30);
    pros::delay(1500);
   // left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(49, -53, 500,{.forwards=false,},false); // back a lil
    chassis.moveToPoint(49, -100, 600,{.maxSpeed=60},false); 
    chassis.moveToPoint(50, 0, 1250,{.forwards=false,.maxSpeed=60},false); // go to goal
    score();
    Unloader.set_value(false);
    chassis.moveToPoint(50, -40, 750); // forwards a bit
    chassis.turnToHeading(230, 750,{.minSpeed=10},false);
    chassis.moveToPoint(33.1,-58.3,750);
    chassis.turnToHeading(265, 750,{.minSpeed=10},false);
    park();
    

    
}

void leftAuton_descore() { //7 ball
    chassis.setPose(0, 0, 0);
    Descore.set_value(true);
    Intake.move(-127);
    chassis.swingToPoint(-10, 20, lemlib::DriveSide::RIGHT, 750);
    chassis.moveToPoint(-10, 20,  2500);
    chassis.turnToHeading(-140, 800);
    chassis.moveToPoint(-35, 0,2000,{.maxSpeed=70},false);
    chassis.turnToHeading(180, 750,{.minSpeed=10},false);
    Unloader.set_value(true);
    chassis.moveToPoint(-31, -100, 500,{.maxSpeed=50},false);
  //  left_motors.move(30);
    //right_motors.move(30);
    pros::delay(800);
    //left_motors.move(0);
    //right_motors.move(0);
    chassis.moveToPoint(-31, 80, 1000,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
    Descore.set_value(false);
        // --- Anti-jam scoring logic (1750ms timeout) ---
    {
        double velocityBuffer[AVG_SIZE] = {0};
        int bufferIndex = 0;

        auto getAvgVelocity = [&]() {
            double sum = 0;
            for (int i = 0; i < AVG_SIZE; i++) sum += velocityBuffer[i];
            return sum / AVG_SIZE;
        };

        uint32_t startTime = pros::millis();

        while (true) {
            velocityBuffer[bufferIndex] = std::abs(Intake.get_actual_velocity());
            bufferIndex = (bufferIndex + 1) % AVG_SIZE;
            double velocity = getAvgVelocity();

            if (velocity <= jam_threshold) {
                Intake.move(96);
                Outtake.move(-96);
                pros::delay(150);
                Intake.move(-127);
                Outtake.move(127);
                pros::delay(500);
            }

            if (velocity >= normal_motor_velocity) {
                pros::delay(1000);
                break;
            }

            if (pros::millis() - startTime >= 1750) {
                break;
            }

            pros::delay(20);
        }
    }
    // --- End scoring ---
    Intake.move(0);
    Outtake.move(0);
    chassis.moveToPoint(-31, 0, 750);
    chassis.turnToHeading(250, 750);
    chassis.moveToPoint(-23, 15, 750,{.forwards=false});
    chassis.turnToHeading(180, 750,{.minSpeed=15});
    chassis.moveToPoint( -23, 30, 9999,{.forwards=false,.minSpeed=80,});

}

void leftAuton() { //mid
    chassis.setPose(0, 0, 0);
    Descore.set_value(true);
    Intake.move(-127);
    chassis.moveToPoint(-10, 20,  2500);
    chassis.turnToPoint(8, 38, 1000,{.forwards=false});
    chassis.moveToPoint(8, 38, 1000,{.forwards=false,.maxSpeed=80},false);
    MidScoring.set_value(true); // score mid
    Intake.move(127);
    pros::delay(200);
    Intake.move(-127);
    Outtake.move(-127);
    pros::delay(2000);
    Outtake.move(0);
    MidScoring.set_value(false);
    chassis.moveToPoint(-35, 10, 2000,{},false); // go to unload plane
    chassis.turnToHeading(180,1000,{},false);
    Unloader.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-35, -40, 800,{.maxSpeed=70},false); // unload
    pros::delay(700);
    Intake.move(0);
    chassis.moveToPoint(-33, 30, 2000,{.forwards=false,.maxSpeed=80},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
    Descore.set_value(false);
    pros::delay(2500);
    Intake.move(0);
    Outtake.move(0);
  //  left_motors.move(40); // Slightly faster
    //right_motors.move(40);
    pros::delay(200); // Reduce
    //left_motors.brake();
    //right_motors.brake();
    pros::delay(300);
    chassis.swingToHeading(330,lemlib::DriveSide::RIGHT,2000,{.minSpeed=50});
    chassis.swingToHeading(15,lemlib::DriveSide::RIGHT,1000,{},false);
    pros::delay(300);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,16.7,1000,{.minSpeed=50});
    
}


void rightAuton() {
    chassis.setPose(0, 0, 0);
    Descore.set_value(true);
    Intake.move(-127);
    chassis.swingToPoint(10, 20, lemlib::DriveSide::RIGHT, 750);
    chassis.moveToPoint(10, 20,  2500);
    chassis.turnToHeading(140, 800);
    chassis.moveToPoint(35, 0,2000,{.maxSpeed=70},false);
    chassis.turnToHeading(180, 750,{.minSpeed=10},false);
    Unloader.set_value(true);
    chassis.moveToPoint(31, -100, 500,{.maxSpeed=60},false);
    left_motors.move(30);
    right_motors.move(30);
    pros::delay(800);
    left_motors.move(0);
    right_motors.move(0);
    chassis.moveToPoint(31, 80, 1000,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
    Descore.set_value(false);

    // --- Anti-jam scoring logic (1750ms timeout) ---
    {
        double velocityBuffer[AVG_SIZE] = {0};
        int bufferIndex = 0;

        auto getAvgVelocity = [&]() {
            double sum = 0;
            for (int i = 0; i < AVG_SIZE; i++) sum += velocityBuffer[i];
            return sum / AVG_SIZE;
        };

        uint32_t startTime = pros::millis();

        while (true) {
            velocityBuffer[bufferIndex] = std::abs(Intake.get_actual_velocity());
            bufferIndex = (bufferIndex + 1) % AVG_SIZE;
            double velocity = getAvgVelocity();

            if (velocity <= jam_threshold) {
                Intake.move(96);
                Outtake.move(-96);
                pros::delay(150);
                Intake.move(-127);
                Outtake.move(127);
                pros::delay(500);
            }

            if (velocity >= normal_motor_velocity) {
                pros::delay(1000);
                break;
            }

            if (pros::millis() - startTime >= 1750) {
                break;
            }

            pros::delay(20);
        }
    }
    // --- End scoring ---

    Intake.move(0);
    Outtake.move(0);
    chassis.moveToPoint(31, 0, 750);
    chassis.turnToHeading(250, 750);
    chassis.moveToPoint(42, 15, 750,{.forwards=false});
    chassis.turnToHeading(180, 750,{.minSpeed=15});
    chassis.moveToPoint( 42, 30, 9999,{.forwards=false,.minSpeed=80,});
}
