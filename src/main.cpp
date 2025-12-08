#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstddef>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

#define MOTOR_PORT 3

// motor groups
// pros::MotorGroup leftMotors({2,-3, 1}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
// pros::MotorGroup rightMotors({-11,20, -12}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//pros::MotorGroup leftMotors({-12,1,-11}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-12,-1,11}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


pros::Motor lowIntake(3,pros::MotorGearset::blue);
pros::Motor topintake(2,pros::MotorGearset::rpm_200);
pros::Motor Outtake(9,pros::MotorGearset::rpm_200);
pros::GPS gps(17);
pros::ADIDigitalOut piston('G');
pros::ADIDigitalOut piston1('C');
pros::ADIDigitalOut piston2('H');





// Inertial Sensor on port 13
pros::Imu imu(8);               

// tracking wheels. 
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(15);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(4);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 4.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 5.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(3.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             45, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
//lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    piston.set_value(true);
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {

    piston.set_value(true);
}


/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {

    piston.set_value(true);
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy


void intakeblocks() {

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

}

void highgoalscoring(){


    piston1.set_value(true);
    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200);

}

void midgoalscoring(){
    piston1.set_value(false);
    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(-200); 

}

void lowgoalscoring(){
    piston1.set_value(false);

    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 
}

void hoodup(){

    piston1.set_value(true);

}

void hooddown(){

    piston1.set_value(false);

}



void rightlong(){
    chassis.setPose(0,0,0);
    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

    chassis.moveToPoint(0, 34,1000,{.maxSpeed=100});

    chassis.turnToHeading(90, 500,{.maxSpeed=100});
  
    piston2.set_value(true);


    chassis.moveToPose(60, 40,90, 1250,{.maxSpeed=1000});


    chassis.moveToPoint(-20, 40,900,{.forwards=false,.maxSpeed=1000});

    pros::delay(750);

    piston1.set_value(true);
	topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    piston2.set_value(false);

}
void rightlongtomiddle(){



    pros::delay(2000);
    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

    chassis.moveToPoint(-2, 39, 750,{.maxSpeed=100});

    chassis.turnToHeading(210, 500);

    chassis.moveToPoint(-12.8, 7.1, 1000,{.maxSpeed=100});
    pros::delay(750);
    piston2.set_value(true);
    chassis.turnToHeading(240, 300);

    chassis.moveToPoint(-28, -6, 800,{.maxSpeed=100});

    piston2.set_value(false);


    piston1.set_value(false);

    pros::delay(750);

    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-200);
    Outtake.move_velocity(-200); 

    pros::delay(1500);



    chassis.moveToPoint(-9, 19, 800,{.forwards=false,.maxSpeed=100});


    chassis.turnToHeading(90,500);

    piston.set_value(true);

    chassis.moveToPoint(-35, 29, 800,{.forwards=false,.maxSpeed=100});

    piston.set_value(false);


    // chassis.moveToPoint(-13, 6,750,{.forwards=false,.maxSpeed=100});

    // topintake.move_velocity(200);   
    // lowIntake.move_velocity(600);
    // Outtake.move_velocity(200); 
    // piston1.set_value(false);


    // chassis.turnToHeading(180, 500);

    // chassis.moveToPoint(-13, -37.5, 1000,{.maxSpeed=100});
    // pros::delay(600);
    // piston2.set_value(true);


    //////////////////

    //chassis.moveToPoint(-9.7, -40.3, 1000,{.maxSpeed=50});


    



    // piston1.set_value(false);

    // topintake.move_velocity(-200); 
    // lowIntake.move_velocity(-600);
    // Outtake.move_velocity(-200); 

    // pros::delay(750);



    // pros::delay(1250);


    


}

void middletoleftlong() {


    



    chassis.turnToHeading(125.4, 500);


    chassis.moveToPoint(10, -56, 1000,{.maxSpeed=100});
    chassis.turnToHeading(90, 500,{.maxSpeed=100});



    chassis.moveToPoint(30, -52, 1000,{.maxSpeed=100});







    chassis.moveToPoint(-13, -54, 1500,{.forwards=false,.maxSpeed=100});

    pros::delay(750);


    piston1.set_value(true);
	topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    piston2.set_value(false);


}

void sevenballright(){

    chassis.setPose(0,0,0);

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

    chassis.moveToPoint(0, 12.5, 300);
    chassis.turnToHeading(25,300);

    chassis.moveToPoint(10.5, 32.7, 1500,{.maxSpeed=30});
    pros::delay(1000);
    piston2.set_value(true);

    chassis.turnToHeading(141.7,500);

    chassis.moveToPoint(41.9, 11, 1000,{.maxSpeed=70});

    chassis.turnToHeading(180,500);




    chassis.moveToPoint(42.4, -20, 1300,{.maxSpeed=1000});

    chassis.moveToPoint(42.0, 25, 1000,{.forwards = false,.maxSpeed=70});

    pros::delay(750);


    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(true);

    pros::delay(1000);



    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 

    pros::delay(250);

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    pros::delay(1250);

    piston1.set_value(false);


    chassis.moveToPoint(43.8, 2, 500);
    chassis.turnToHeading(138, 500);

    chassis.moveToPoint(33.1, 23.5, 750,{.forwards=false});

    chassis.turnToHeading(180, 750);
    piston.set_value(false);

    chassis.moveToPoint(34, 42, 2000,{.forwards=false,.maxSpeed=100});
    





    

    

    //piston.set_value(false);




}

void skillsfirstgoal(){

    chassis.setPose(0,0,0);

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

    chassis.moveToPoint(0, 11.5, 300);
    chassis.turnToHeading(90,500);


    chassis.moveToPoint(39, 13, 1500,{.maxSpeed=50});

    chassis.turnToHeading(180,1000);

    piston2.set_value(true);

    pros::delay(500);

    //start first scraping
    chassis.moveToPoint(42, -20, 3000,{.maxSpeed=1000});

    //come back from first goal scraping


    chassis.moveToPoint(42, 6.7, 1000,{.forwards = false,.maxSpeed=50});

    pros::delay(1000);



    piston2.set_value(false);

    chassis.turnToHeading(90, 500);

    chassis.moveToPoint(53, 11, 750);


    chassis.turnToHeading(11, 750);

    chassis.moveToPoint(47, 23,750);

    chassis.turnToHeading(0, 750);



    // ally stretch
    chassis.moveToPoint(46, 87,2500,{.maxSpeed=60});



    chassis.turnToHeading(-45,500);
    chassis.moveToPoint(33.6, 90,1000,{.maxSpeed=60});

    chassis.turnToHeading(0, 500);


    chassis.moveToPoint(33, 73,1000,{.forwards=false,.maxSpeed=60});

    //first scoring

    pros::delay(750);





    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 

    pros::delay(250);

    //open the gate 
    hoodup();

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    pros::delay(1500);

    //scrapper up
    piston2.set_value(true);

    pros::delay(500);
    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 
    pros::delay(250);

    hooddown();

    intakeblocks();

    // second scrapping 
    chassis.moveToPoint(32, 130, 3000,{.maxSpeed=70});

    chassis.moveToPoint(33, 75,1000,{.forwards=false,.maxSpeed=60});





    pros::delay(750);


    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    hoodup();

    pros::delay(1000);



    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 

    pros::delay(250);

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    pros::delay(1250);


    piston2.set_value(false);

    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 

    pros::delay(400);

    hooddown();

    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 


    chassis.moveToPoint(32, 90,1000,{.maxSpeed=60});

    chassis.moveToPoint(32, 50,1000,{.forwards=false,.maxSpeed=50});



    
    hoodup();
    chassis.moveToPoint(32, 90,1000,{.maxSpeed=60});




    chassis.turnToHeading(-90, 1000);
    hooddown();


    chassis.moveToPoint(-56, 90,2000,{.maxSpeed=90});
    chassis.turnToHeading(0, 500);

    piston2.set_value(true);
    /// x = -69
    chassis.moveToPoint(-49, 120,3000,{.maxSpeed=80});



    chassis.moveToPoint(-62, 93,1000,{.forwards=false,.maxSpeed=50});



    chassis.turnToHeading(-90, 500);

    piston2.set_value(false);


    chassis.moveToPoint(-69, 88,500,{.maxSpeed=50});

    chassis.turnToHeading(180, 750);


    chassis.moveToPoint(-65, 50,2000,{.maxSpeed=80});

    chassis.turnToHeading(170, 750);

    chassis.moveToPoint(-62, 13,2000,{.maxSpeed=80});




    chassis.turnToHeading(-235, 500);
    // /////////

    chassis.moveToPoint(-52, 32,500,{.maxSpeed=80});

    chassis.turnToHeading(-180, 500);



    chassis.moveToPoint(-49, 25,1000,{.forwards=false,.maxSpeed=50}); // Scoring 

    pros::delay(500);

    piston1.set_value(true);
    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 



    pros::delay(2000);


    topintake.move_velocity(200);   
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 
    piston1.set_value(false);

    piston2.set_value(true);

    chassis.moveToPoint(-49, -20,3000,{.maxSpeed=80});

    chassis.moveToPoint(-49, 40,1000,{.forwards=false,.maxSpeed=50});
    pros::delay(750);


    piston1.set_value(true);
    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    pros::delay(2000);

    topintake.move_velocity(-200); 
    lowIntake.move_velocity(-600);
    Outtake.move_velocity(-200); 

    pros::delay(400);

    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(200); 

    pros::delay(1000);


    

    piston2.set_value(false);


    chassis.moveToPoint(-49, 16,3000,{.maxSpeed=80});


    chassis.turnToHeading(90, 500);

    chassis.moveToPoint(-16, 19,1500,{.maxSpeed=80});

    chassis.turnToHeading(180, 500);

    chassis.moveToPoint(-16, 40,500,{.forwards=false,.maxSpeed=50});

    chassis.moveToPoint(-16, -35,4000,{.maxSpeed=500000});











    // chassis.moveToPoint(20, 89,1000,{.maxSpeed=60});

    // chassis.turnToHeading(-180, 1000);


    // chassis.moveToPoint(20, 5,4000,{.maxSpeed=80});


    // chassis.turnToHeading(-90,1000);

    // chassis.moveToPoint(-12, 3,4000,{.maxSpeed=80});
    // chassis.turnToHeading(-180,1000);

    // chassis.moveToPoint(-8, 14,1000,{.forwards= false,.maxSpeed=80});
    // chassis.moveToPoint(-7, -200,2000,{.maxSpeed=1000000});

    



}

void solopp(){
    //rightlong();
    //rightlongtomiddle();
    // middletoleftlong();

    //sevenballright();

    skillsfirstgoal();


}

void autonomous() {

    solopp();


    // chassis.moveToPoint(21.85, -28, 2000,{.forwards = false, .minSpeed=60});
    // piston1.set_value(true);
    // topintake.move_velocity(200); 
    // lowIntake.move_velocity(600);
    // Outtake.move_velocity(200); 

    // piston2.set_value(false);
    // pros::delay(1000);


    // topintake.move_velocity(200);   
    // lowIntake.move_velocity(600);
    // Outtake.move_velocity(200); 
    // piston1.set_value(false);

    // chassis.turnToHeading(180, 500);


    // chassis.turnToHeading(90, 1000,{.minSpeed=50});

    // chassis.moveToPoint(16, 42,  2000,{.minSpeed=75});x
    // pros::delay(2000);
    // chassis.moveToPoint(7, 42,  2000, {.forwards=false,.minSpeed=50});

    // chassis.turnToHeading(0, 1000,{.minSpeed=50});

    // chassis.moveToPoint(0, 43,  2000);



    // //chassis.moveToPose(-8, 43,0, 3000);
    // chassis.turnToHeading(-90, 1000,{.minSpeed=30});

    // chassis.moveToPose(-77, 35,-90,  2000,{.minSpeed=75});


    // chassis.turnToHeading(-149, 1000,{.minSpeed=50});

}
/**
 * Runs in driver control
 */
int pistonvalue = true;
void opcontrol() {
    


    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX, false, 0.75);
        // delay to save resources
        pros::delay(25);
		


		if (controller.get_digital(DIGITAL_L1)) {

            piston1.set_value(false);
			topintake.move_velocity(200); 
            lowIntake.move_velocity(600);
            Outtake.move_velocity(-200); 
           

		}
		else if (controller.get_digital(DIGITAL_L2)) {

			topintake.move_velocity(200);   
            lowIntake.move_velocity(600);
            Outtake.move_velocity(200); 
            piston1.set_value(false);

		}
        else if (controller.get_digital(DIGITAL_R1)) {


            piston1.set_value(false);

			topintake.move_velocity(-200); 
            lowIntake.move_velocity(-600);
            Outtake.move_velocity(-200); 
		}
		else if (controller.get_digital(DIGITAL_R2)) {

            piston1.set_value(true);
			topintake.move_velocity(200); 
            lowIntake.move_velocity(600);
            Outtake.move_velocity(200); 

		}
		else {
			topintake.move_velocity(0); 
            lowIntake.move_velocity(0);
            Outtake.move_velocity(0);
		}
        
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

		
		pros::delay(2);

    }

}

	