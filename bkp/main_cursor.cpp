#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

#define MOTOR_PORT 3

// Motor groups
pros::MotorGroup leftMotors({-12, -1, 11}, pros::MotorGearset::blue); // left
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue); // right

pros::Motor lowIntake(13, pros::MotorGearset::blue);
pros::Motor topintake(14, pros::MotorGearset::rpm_200);
pros::GPS gps(17);
pros::adi::DigitalOut piston('G');
pros::adi::DigitalOut piston1('H');

// Inertial Sensor on port 2
pros::Imu imu(2);

// Tracking wheels
pros::Rotation horizontalEnc(15); // Not reversed
pros::Rotation verticalEnc(-16);  // Reversed
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 4.5);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.5);

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    16, // track width
    lemlib::Omniwheel::NEW_275,
    600, // rpm
    6 // horizontal drift
);

// PID controllers
lemlib::ControllerSettings linearController(6, 0, 40, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angularController(1.5, 0, 10, 0, 0, 0, 0, 0, 0);

// Odometry sensors
lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// --- Feedback with GPS sensor integration for higher-accuracy correction ---
/**
 * Moves to a target (x, y) using standard odometry and PID,
 * but periodically checks GPS and corrects if error grows too large.
 * Units: LemLib default (inches), but GPS returns mm. Conversion: 1 in = 25.4 mm
 *
 * @param x_in     Target X (inches)
 * @param y_in     Target Y (inches)
 * @param timeout  Timeout in ms
 * @param gpsTolerance_mm  Acceptable GPS error (for example: 30mm)
 * @param correctionInterval  How often to check GPS in ms (default: 150ms)
 */
void moveToPointWithGpsFeedback(double x_in, double y_in, int timeout = 2000, double gpsTolerance_mm = 30, int correctionInterval = 150) {
    // Set the desired target using LemLib odometry/PID
    chassis.moveToPoint(x_in, y_in, timeout);

    // Wait a short moment to allow chassis to start moving
    pros::delay(70);

    // Main correction loop
    int elapsed = 0;
    while (elapsed < timeout) {
        // Get odom pose and GPS pose (mm)
        lemlib::Pose odomPose = chassis.getPose();
        double gps_x_mm = gps.get_position_x();
        double gps_y_mm = gps.get_position_y();

        // Convert odom to mm
        double odom_x_mm = odomPose.x * 25.4;
        double odom_y_mm = odomPose.y * 25.4;

        // Compute the difference
        double x_err = gps_x_mm - odom_x_mm;
        double y_err = gps_y_mm - odom_y_mm;
        double gps_err = std::sqrt(x_err * x_err + y_err * y_err);

        // If GPS error is large, correct odom
        if (gps_err > gpsTolerance_mm && std::abs(gps_x_mm) > 0.1 && std::abs(gps_y_mm) > 0.1) {
            // Reset chassis odometry to match GPS reading (convert mm to inches)
            chassis.setPose(gps_x_mm / 25.4, gps_y_mm / 25.4, odomPose.theta); // Use previous theta
        }

        // Check if we're close to target (in GPS); if so, break out
        double target_x_mm = x_in * 25.4;
        double target_y_mm = y_in * 25.4;
        double dist_to_target_gps = std::sqrt((gps_x_mm - target_x_mm) * (gps_x_mm - target_x_mm) + (gps_y_mm - target_y_mm) * (gps_y_mm - target_y_mm));
        if (dist_to_target_gps < gpsTolerance_mm) {
            break;
        }

        pros::delay(correctionInterval);
        elapsed += correctionInterval;
    }

    // Brief stop to ensure motors halt at the end
    chassis.cancelMotion();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // Screen/telemetry thread
    pros::Task* screenTask = new pros::Task([]() {
        while (true) {
            // Odom and GPS
            lemlib::Pose p = chassis.getPose();
            pros::lcd::print(0, "Odom X: %.2f", p.x);
            pros::lcd::print(1, "Odom Y: %.2f", p.y);
            pros::lcd::print(2, "Odom Th: %.2f", p.theta);
            pros::lcd::print(3, "GPS_X: %.1f", gps.get_position_x());
            pros::lcd::print(4, "GPS_Y: %.1f", gps.get_position_y());
            pros::lcd::print(5, "GPS_Head: %.2f", gps.get_heading());
            pros::screen::print(::pros::E_TEXT_MEDIUM, 3, "Odom X: %.2f", p.x);

            lemlib::telemetrySink()->info("Chassis pose: {}", p);
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

// get a path used for pure pursuit
// Note: Remove or comment out ASSET macro if it's not defined in your build!
/* ASSET(example_txt); */

/**
 * Runs during auto
 *
 * Now uses feedback via GPS sensor for correction.
 */
void autonomous() {
    // Set initial odometry position to GPS, if sensor valid
    double gx = gps.get_position_x(), gy = gps.get_position_y();
    if (std::abs(gx) > 2.0 && std::abs(gy) > 2.0) {
        chassis.setPose(gx / 25.4, gy / 25.4, chassis.getPose().theta);
    } else {
        chassis.setPose(0, 0, 0);
    }

    // Example - move to 6,20 inches with live GPS feedback (+-30mm tolerance)
    moveToPointWithGpsFeedback(6, 20, 2000, 30, 100);
} 

void opcontrol() {
    //autonomous();
    //return;

    // controller loop
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lowIntake.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lowIntake.move_velocity(-600);
        } else {
            lowIntake.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            topintake.move_velocity(200); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            topintake.move_velocity(-200);
        } else {
            topintake.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            piston.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            piston.set_value(false);
        }

        pros::delay(25);
    }
}
