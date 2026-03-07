#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstddef>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive Motors (6-motor configuration)
pros::MotorGroup leftMotors({-12, -1, 11}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue);

// Intake & Scoring Motors
pros::Motor lowIntake(3, pros::MotorGearset::blue);      // 600 RPMx
pros::Motor topintake(2, pros::MotorGearset::rpm_200);   // 200 RPM
pros::Motor Outtake(9, pros::MotorGearset::rpm_200);     // 200 RPM

// Sensors
pros::GPS gps(17);
pros::Imu imu(8);
pros::Rotation verticalEnc(4);

// Pneumatics
pros::ADIDigitalOut piston('G');   // Main deployment
pros::ADIDigitalOut piston1('C');  // Hood control
pros::ADIDigitalOut piston2('H');  // Scraper

// ============================================================================
// TRACKING WHEEL CONFIGURATION
// ============================================================================

// Vertical tracking wheel: 2.75" diameter, 5.5" offset from center
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 5.5);

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    13.5,                        // Track width (inches)
    lemlib::Omniwheel::NEW_275,  // Wheel type
    600,                         // Drivetrain RPM
    8                            // Horizontal drift correction
);

// ============================================================================
// PID CONTROLLERS - FIXED WITH PROPER TUNING
// ============================================================================

// Lateral (forward/backward) PID - FIXED
lemlib::ControllerSettings linearController(
    3.5,   // kP - Proportional gain
    0,     // kI - Integral gain
    1.5,   // kD - Derivative gain (ADDED - was 0, caused overshoot)
    3,     // Anti-windup
    1,     // Small error range (inches) - FIXED (was 0)
    100,   // Small error timeout (ms) - FIXED (was 0)
    3,     // Large error range (inches) - FIXED (was 0)
    500,   // Large error timeout (ms) - FIXED (was 0)
    20     // Slew rate (acceleration limit)
);

// Angular (turning) PID - FIXED
lemlib::ControllerSettings angularController(
    5,     // kP - Proportional gain
    0,     // kI - Integral gain
    35,    // kD - Derivative gain (REDUCED from 45 to prevent oscillation)
    3,     // Anti-windup
    1,     // Small error range (degrees) - FIXED (was 0)
    100,   // Small error timeout (ms) - FIXED (was 0)
    3,     // Large error range (degrees) - FIXED (was 0)
    500,   // Large error timeout (ms) - FIXED (was 0)
    0      // Slew rate
);

// Odometry sensors
lemlib::OdomSensors sensors(
    &vertical,  // Vertical tracking wheel
    nullptr,    // No second vertical wheel
    nullptr,    // No horizontal wheel
    nullptr,    // No second horizontal wheel
    &imu        // Inertial sensor
);

// Drive curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Create chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// ============================================================================
// SUBSYSTEM CONTROL FUNCTIONS
// ============================================================================

// Intake blocks - run all motors forward
void intakeblocks() {
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
    piston1.set_value(false);  // Hood down for intake
}

// Stop all intake motors
void stopIntake() {
    topintake.move_velocity(0);
    lowIntake.move_velocity(0);
    Outtake.move_velocity(0);
}

// Score to high goal
void highgoalscoring() {
    piston1.set_value(true);   // Hood UP
    pros::delay(250);          // FIXED: Wait for hood to raise
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
}

// Score to mid goal
void midgoalscoring() {
    piston1.set_value(false);  // Hood down
    pros::delay(150);          // Wait for hood
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(-200);  // Reverse outtake for mid
}

// Score to low goal (reverse everything)
void lowgoalscoring() {
    piston1.set_value(false);
    topintake.move_velocity(-200);
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200);
}

// Hood control
void hoodup() {
    piston1.set_value(true);
    pros::delay(200);  // FIXED: Wait for hood to fully raise
}

void hooddown() {
    piston1.set_value(false);
    pros::delay(150);  // FIXED: Wait for hood to lower
}

// Reverse intake briefly to clear jams
void clearJam() {
    topintake.move_velocity(-200);
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200);
    pros::delay(250);
}

// ============================================================================
// AUTONOMOUS ROUTINES
// ============================================================================

/**
 * Skills Autonomous - First Goal Route (FIXED)
 * Starting position: Next to parking corner
 *
 * Route:
 * 1. Scrape first loader
 * 2. Drive to ally stretch, collect blocks
 * 3. Score in first long goal
 * 4. Cross field to second goal
 * 5. Scrape second loader
 * 6. Score in second long goal
 * 7. Final push to center
 */
void skillsfirstgoal_fixed() {
    // Set starting position (adjust based on actual parking corner position)
    // If parking corner is bottom-left, robot faces toward field
    chassis.setPose(0, 0, 0);

    // Start intake
    intakeblocks();

    // ========== PHASE 1: Initial approach to loader ==========
    chassis.moveToPoint(0, 11.5, 500);
    chassis.turnToHeading(90, 500);

    // Move to loader position
    chassis.moveToPoint(39, 13, 1500, {.maxSpeed = 50});
    chassis.turnToHeading(180, 1000);

    // Deploy scraper
    piston2.set_value(true);
    pros::delay(500);

    // ========== PHASE 2: First loader scrape ==========
    chassis.moveToPoint(42, -20, 3000, {.maxSpeed = 100});  // FIXED: was maxSpeed=1000

    // Return from scrape
    chassis.moveToPoint(42, 6.7, 1000, {.forwards = false, .maxSpeed = 50});
    pros::delay(500);

    // Retract scraper, reposition
    piston2.set_value(false);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(53, 11, 750);
    chassis.turnToHeading(11, 750);
    chassis.moveToPoint(47, 23, 750);
    chassis.turnToHeading(0, 750);

    // ========== PHASE 3: Ally stretch collection ==========
    chassis.moveToPoint(46, 87, 2500, {.maxSpeed = 60});

    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(33.6, 90, 1000, {.maxSpeed = 60});
    chassis.turnToHeading(0, 500);

    // Approach first goal for scoring
    chassis.moveToPoint(33, 73, 1000, {.forwards = false, .maxSpeed = 60});
    pros::delay(500);

    // ========== PHASE 4: First scoring sequence ==========
    // Clear any jammed blocks
    clearJam();

    // Raise hood and score
    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
    pros::delay(1500);

    // Deploy scraper for next collection
    piston2.set_value(true);
    pros::delay(500);

    // Clear and continue
    clearJam();
    hooddown();
    intakeblocks();

    // ========== PHASE 5: Second loader scrape ==========
    chassis.moveToPoint(32, 130, 3000, {.maxSpeed = 70});
    chassis.moveToPoint(33, 75, 1000, {.forwards = false, .maxSpeed = 60});
    pros::delay(500);

    // ========== PHASE 6: Second scoring sequence ==========
    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
    pros::delay(1000);

    // Clear jam and continue
    clearJam();
    intakeblocks();
    pros::delay(1250);

    piston2.set_value(false);
    clearJam();
    hooddown();
    intakeblocks();

    // Extra scoring push
    chassis.moveToPoint(32, 90, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(32, 50, 1000, {.forwards = false, .maxSpeed = 50});

    hoodup();
    chassis.moveToPoint(32, 90, 1000, {.maxSpeed = 60});

    // ========== PHASE 7: Cross field to second goal ==========
    chassis.turnToHeading(-90, 1000);
    hooddown();

    chassis.moveToPoint(-56, 90, 2000, {.maxSpeed = 90});
    chassis.turnToHeading(0, 500);

    // Scrape second side loader
    piston2.set_value(true);
    chassis.moveToPoint(-60, 120, 3000, {.maxSpeed = 80});
    chassis.moveToPoint(-60, 93, 1000, {.forwards = false, .maxSpeed = 50});

    chassis.turnToHeading(-90, 500);
    piston2.set_value(false);

    chassis.moveToPoint(-69, 88, 500, {.maxSpeed = 50});
    chassis.turnToHeading(180, 750);

    // Collect blocks on way to goal
    chassis.moveToPoint(-65, 50, 2000, {.maxSpeed = 80});
    chassis.turnToHeading(170, 750);
    chassis.moveToPoint(-62, 13, 2000, {.maxSpeed = 80});

    chassis.turnToHeading(-235, 500);
    chassis.moveToPoint(-51, 15, 500, {.maxSpeed = 80});
    chassis.turnToHeading(-180, 500);

    // ========== PHASE 8: Score in second goal ==========
    chassis.moveToPoint(-51, 25, 1000, {.forwards = false, .maxSpeed = 50});
    pros::delay(500);

    // Score
    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
    pros::delay(2000);

    // Prepare for next scrape
    hooddown();
    intakeblocks();
    piston2.set_value(true);

    // Final loader scrape
    chassis.moveToPoint(-51, -20, 3000, {.maxSpeed = 80});
    chassis.moveToPoint(-51, 40, 1000, {.forwards = false, .maxSpeed = 50});
    pros::delay(500);

    // Final scoring
    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);
    pros::delay(2000);

    // Clear and final push
    clearJam();
    intakeblocks();
    pros::delay(1000);

    piston2.set_value(false);

    // ========== PHASE 9: Center push ==========
    chassis.moveToPoint(-51, 16, 1000, {.maxSpeed = 80});
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(-13, 19, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(180, 500);

    // Back up to prepare for final push
    chassis.moveToPoint(-13, 40, 500, {.forwards = false, .maxSpeed = 50});

    // Final push toward goal - FIXED: maxSpeed was 500000!
    chassis.moveToPoint(-13, -35, 4000, {.maxSpeed = 100});
}

// ============================================================================
// MATCH AUTONOMOUS ROUTINES (15 seconds)
// ============================================================================

void rightlong_fixed() {
    chassis.setPose(0, 0, 0);

    intakeblocks();

    chassis.moveToPoint(0, 34, 1000, {.maxSpeed = 100});
    chassis.turnToHeading(90, 500, {.maxSpeed = 100});

    piston2.set_value(true);
    chassis.moveToPose(60, 40, 90, 1250, {.maxSpeed = 100});  // FIXED: was maxSpeed=1000

    chassis.moveToPoint(-20, 40, 900, {.forwards = false, .maxSpeed = 100});  // FIXED
    pros::delay(750);

    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);

    piston2.set_value(false);
}

void sevenballright_fixed() {
    chassis.setPose(0, 0, 0);

    intakeblocks();

    chassis.moveToPoint(0, 12.5, 300);
    chassis.turnToHeading(25, 300);

    chassis.moveToPoint(10.5, 32.7, 1500, {.maxSpeed = 30});
    pros::delay(1000);
    piston2.set_value(true);

    chassis.turnToHeading(141.7, 500);
    chassis.moveToPoint(41.9, 11, 1000, {.maxSpeed = 70});
    chassis.turnToHeading(180, 500);

    chassis.moveToPoint(42.4, -20, 1300, {.maxSpeed = 100});  // FIXED: was maxSpeed=1000
    chassis.moveToPoint(42.0, 25, 1000, {.forwards = false, .maxSpeed = 70});

    pros::delay(750);

    hoodup();
    topintake.move_velocity(200);
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);

    pros::delay(1000);

    // Clear jam
    clearJam();
    intakeblocks();
    pros::delay(1250);

    hooddown();

    chassis.moveToPoint(43.8, 2, 500);
    chassis.turnToHeading(138, 500);
    chassis.moveToPoint(33.1, 23.5, 750, {.forwards = false});

    chassis.turnToHeading(180, 750);
    piston.set_value(false);

    chassis.moveToPoint(34, 42, 2000, {.forwards = false, .maxSpeed = 100});
}

// ============================================================================
// MAIN AUTONOMOUS SELECTOR
// ============================================================================

void autonomous() {
    // Run the fixed skills autonomous
    skillsfirstgoal_fixed();

    // Alternative routines (uncomment to use):
    // rightlong_fixed();
    // sevenballright_fixed();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initialize() {
    piston.set_value(true);
    pros::lcd::initialize();
    chassis.calibrate();

    // Screen task for debugging
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {
    piston.set_value(true);
}

void competition_initialize() {
    piston.set_value(true);
}

// ============================================================================
// DRIVER CONTROL
// ============================================================================

bool pistonvalue = true;

void opcontrol() {
    while (true) {
        // Arcade drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        // Intake controls
        if (controller.get_digital(DIGITAL_L1)) {
            midgoalscoring();
        }
        else if (controller.get_digital(DIGITAL_L2)) {
            intakeblocks();
        }
        else if (controller.get_digital(DIGITAL_R1)) {
            lowgoalscoring();
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            highgoalscoring();
        }
        else {
            stopIntake();
        }

        // Piston controls
        if (controller.get_digital(DIGITAL_UP)) {
            piston.set_value(pistonvalue);
            pistonvalue = !pistonvalue;
            pros::delay(250);
        }
        if (controller.get_digital(DIGITAL_DOWN)) {
            piston.set_value(false);
        }
        if (controller.get_digital(DIGITAL_X)) {
            piston2.set_value(true);
        }
        if (controller.get_digital(DIGITAL_B)) {
            piston2.set_value(false);
        }

        pros::delay(20);
    }
}