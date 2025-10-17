#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstddef>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

#define MOTOR_PORT 3

// motor groups
pros::MotorGroup leftMotors({-12,-1,11}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue);

pros::Motor lowIntake(13,pros::MotorGearset::blue);
pros::Motor topintake(14,pros::MotorGearset::rpm_200);
pros::GPS gps(17);
pros::ADIDigitalOut piston('G');
pros::ADIDigitalOut piston1('H');

// Inertial Sensor on port 2
pros::Imu imu(2);

// tracking wheels
pros::Rotation horizontalEnc(15);
pros::Rotation verticalEnc(-16);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 4.5);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              16, // track width
                              lemlib::Omniwheel::NEW_275,
                              600, // drivetrain rpm
                              6 // horizontal drift
);

// lateral motion controller
lemlib::ControllerSettings linearController(6, 0, 40, 0, 0, 0, 0, 0, 0);

// angular motion controller
lemlib::ControllerSettings angularController(1.5, 0, 10, 0, 0, 0, 0, 0, 0);

//sensors for odometry
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

// input curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code.
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

ASSET(example_txt);

void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(25);
    }
}
