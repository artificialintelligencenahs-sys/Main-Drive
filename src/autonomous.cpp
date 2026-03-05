#include "autonomous.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "helper_functions.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <ctime>

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
    left_motors.move(30);
    right_motors.move(30);
    pros::delay(600);
    left_motors.move(0);
    right_motors.move(0);
    chassis.moveToPoint(30, 30, 1000,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
    pros::delay(600);
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
    chassis.moveToPoint(-68, 10, 2000);
    chassis.turnToHeading(180, 500,{},false);
    Unloader.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-68, -100, 500,{.maxSpeed=70},false);
    left_motors.move(30);
    right_motors.move(30);
    pros::delay(600);
    left_motors.move(0);
    right_motors.move(0);
    chassis.moveToPoint(-67, 30, 750,{.forwards=false,.maxSpeed=70},false);
    Intake.move(-127);
    Outtake.move(127);
    Unloader.set_value(false);
}



void skills_auton() { 

}

void leftAuton_descore() { //7 ball
    chassis.setPose(0, 0, 0);
    Descore.set_value(true);
    Intake.move(-127);
    chassis.moveToPoint(-10, 20,  2500);
    chassis.turnToHeading(180, 800);
    chassis.moveToPose(-30, 0, 270,2000,{.minSpeed=50},false);
    chassis.turnToPoint(-30, -10, 800);
    Unloader.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-30, -30, 800,{.maxSpeed=70},false); // unload
    pros::delay(400);
    Intake.move(0);
    chassis.moveToPoint(-30, 30, 1500,{.forwards=false,.maxSpeed=80},false);
    Intake.move(-127);
    Outtake.move(-127);
    Unloader.set_value(false);
    Descore.set_value(false);
    pros::delay(2500);
    Intake.move(0);
    Outtake.move(0);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,800);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-7, 12, 800,{.forwards=false,.minSpeed=50},false);
    chassis.turnToHeading(0, 750,{},false);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -30, 9500,{.forwards=false});
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
    left_motors.move(40); // Slightly faster
    right_motors.move(40);
    pros::delay(200); // Reduce
    left_motors.brake();
    right_motors.brake();
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
    chassis.moveToPoint(10, 20,  2500);
    chassis.turnToHeading(180, 800);
    chassis.moveToPose(30, 0, 90,2000,{.minSpeed=50},false);
    chassis.turnToPoint(30, -10, 800);
    Unloader.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(30, -30, 800,{.maxSpeed=70},false); // unload
    pros::delay(400);
    Intake.move(0);
    chassis.moveToPoint(30, 30, 1500,{.forwards=false,.maxSpeed=80},false);
    Intake.move(-127);
    Outtake.move(-127);
    Unloader.set_value(false);
    Descore.set_value(false);
    pros::delay(2500);
    Intake.move(0);
    Outtake.move(0);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,12,800);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-7, 12, 800,{.forwards=false,.minSpeed=50},false);
    chassis.turnToHeading(0, 750,{},false);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -30, 9500,{.forwards=false});

}
