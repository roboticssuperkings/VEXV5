#include "lemlib/api.hpp"
#include "main.h" // Assuming your GPS sensor setup is handled here

// --- Define LemLib Chassis and Components ---

// Motor Groups
pros::MotorGroup leftMotors({-12, -1, 11}, pros::MotorGearset::blue); // left
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue); // right

// Other Devices
pros::Motor lowIntake(13, pros::MotorGearset::blue);
pros::Motor topintake(14, pros::MotorGearset::rpm_200);
pros::GPS gps(17);
pros::ADIDigitalOut piston('G');
pros::ADIDigitalOut piston1('H');
pros::Imu imu(2);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Tracking Wheels
pros::Rotation horizontalEnc(15); // Not reversed
pros::Rotation verticalEnc(-16);  // Reversed
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 4.5);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.5);

// Drivetrain and Chassis Setup
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    16, // track width, inches
    lemlib::Omniwheel::NEW_275,
    600, // rpm
    6 // horizontal drift
);

lemlib::ControllerSettings linearController(6, 0, 40, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angularController(1.5, 0, 10, 0, 0, 0, 0, 0, 0);

lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// --- Helper Functions for GPS Conversion (mm to inches) ---
double get_gps_x() {
    // Convert GPS x from mm to inches
    return gps.get_position_x() / 25.4;
}

double get_gps_y() {
    // Convert GPS y from mm to inches
    return gps.get_position_y() / 25.4;
}

// --- Example auto function to go to 2 points ---
void autonomous_example_goto_points() {
    // Set starting pose based on GPS or default (as in opcontrol)
    double x_init = get_gps_x();
    double y_init = get_gps_y();
    double theta_init = 0.0;

    if (std::abs(x_init) > 0.1 && std::abs(y_init) > 0.1) {
        chassis.setPose(x_init, y_init, theta_init);
    } else {
        chassis.setPose(0.0, 0.0, theta_init);
    }

    // Example points (in inches, based on field dimensions)
    double point1_x = 24.0; // 2 feet forward
    double point1_y = 0.0;
    double point2_x = 24.0;
    double point2_y = 24.0; // 2 feet to the right

    // Go to first point (24,0)
    chassis.moveTo(point1_x, point1_y, 1000, false, 40); // (x, y, timeout_ms, waitForCompletion, maxSpeed)
    pros::delay(200); // short pause

    // Go to second point (24,24)
    chassis.moveTo(point2_x, point2_y, 1000, false, 40);
    pros::delay(200); // short pause

    // Done. You may add more logic as needed.
}

// --- opcontrol Implementation ---
void opcontrol() {
    // Set initial pose using GPS data (converted to inches)
    double x_init = get_gps_x();
    double y_init = get_gps_y();

    // If the GPS is reporting plausible value, use it; otherwise, fall back to (0,0)
    if (std::abs(x_init) > 0.1 && std::abs(y_init) > 0.1) {
        chassis.setPose(x_init, y_init, 0.0); // Assume 0 heading at start
    } else {
        chassis.setPose(0.0, 0.0, 0.0);
    }

    // EXAMPLE: Run autonomous_example_goto_points() with A button
    bool autonomous_ran = false;

    while (true) {
        // Optionally, trigger 2-point example with A button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !autonomous_ran) {
            autonomous_ran = true;
            autonomous_example_goto_points();
        }

        // Periodically update LemLib's pose with live GPS data
        double current_gps_x = get_gps_x();
        double current_gps_y = get_gps_y();

        // Only update pose if GPS is giving plausible data
        if (std::abs(current_gps_x) > 0.1 && std::abs(current_gps_y) > 0.1) {
            // Use previous heading (theta) from odometry
            lemlib::Pose curPose = chassis.getPose();
            chassis.setPose(current_gps_x, current_gps_y, curPose.theta);
        }

        // Controller drive (arcade as example)
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Intakes
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

        // Pistons
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            piston.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            piston.set_value(false);
        }

        pros::delay(20); // Small delay for loop stability
    }
}
