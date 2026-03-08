#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/gps.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstddef>
#include <cstdint>
#include <future>

// Todo: Add Sensor Fusion Code - Test this
// git add . 
// git commit -m "Comments"
// git push 

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

#define MOTOR_PORT 3

// motor groups
// pros::MotorGroup leftMotors({2,-3, 1}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
// pros::MotorGroup rightMotors({-11,20, -12}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//pros::MotorGroup leftMotors({-12,1,-11}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-12, -1, 11}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


pros::Motor lowIntake(13,pros::MotorGearset::blue);
pros::Motor topintake(6,pros::MotorGearset::blue);
pros::Motor indexer(15,pros::MotorGearset::blue);
pros::Motor Outtake(9,pros::MotorGearset::rpm_200);
pros::GPS gps(17);

pros::ADIDigitalOut piston('G'); // scrapper
pros::ADIDigitalOut piston1('H'); // decore wing
pros::ADIDigitalOut piston2('C'); //middle goal descore
pros::ADIDigitalOut piston3('B'); //middle goal descore



pros::Distance distanceSensor(5);
pros::Distance distanceSensorfront(14);



// Inertial Sensor on port 13
pros::Imu imu(7);               

// tracking wheels. 
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(15);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 4.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
//KP : 3.7 KD 0
// 4 KP ,0 KD  is the optimal value
lemlib::ControllerSettings linearController(4, // proportional gain (kP) 3.7
                                            0, // integral gain (kI)
                                            20, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large er ror range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(6.5 , // proportional gain (kP)
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
            pros::lcd::print(3, "front: %f", distanceSensor.get_distance()); // x
            pros::lcd::print(4, "back: %f",distanceSensorfront.get_distance()); // y
            // pros::lcd::print(5, "rpm: %f", leftMotors.get_actual_velocity());
            // pros::lcd::print(6, "rpm: %f", rightMotors.get_actual_velocity());
            // pros::lcd::print(7, "temp: %f", rightMotors.get_temperature());
			
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


void intakeBlocks() {

    topintake.move_velocity(0); 
    lowIntake.move_velocity(-600);
   

}

void stopIntakeBlocks() {
    topintake.move_velocity(0);   
    lowIntake.move_velocity(0);
    Outtake.move_velocity(0); 

}

void goalHighScoring(){


    
    topintake.move_velocity(-600); 
    lowIntake.move_velocity(-600);


}

void midgoalscoring(){
    piston1.set_value(false);
    topintake.move_velocity(200); 
    lowIntake.move_velocity(600);
    Outtake.move_velocity(-200); 

}

void lowgoalscoring(){

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

void scraperDown(){
    piston2.set_value(true);
}

void scraperUp(){
    piston2.set_value(false);
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

    intakeBlocks();

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

void fourballrightrush(){

    chassis.setPose(0,0,0);

    intakeBlocks();

    chassis.moveToPoint(0, 39, 750);
    chassis.turnToHeading(90,750,{.maxSpeed=80});
    pros::delay(100);

    piston.set_value(false);

    chassis.moveToPoint(25, 34, 1750);

    chassis.moveToPoint(-20, 33, 1000, {.forwards=false, .minSpeed=70});

    piston.set_value(true);

    pros::delay(1000);

    goalHighScoring();

    pros::delay(2000);

    chassis.moveToPoint(-6, 32, 750);

    chassis.turnToHeading(44,750,{.maxSpeed=50});
    chassis.moveToPoint(-11, 27, 750, {.forwards=false, .minSpeed=70});
    chassis.turnToHeading(90, 750);

    piston1.set_value(true);

    chassis.moveToPoint(-24, 25, 1000, {.forwards=false, .minSpeed=70});
}

void distancesensortest(){
    chassis.setPose(0,0,0);


//     while(true) {
//         int dist = distanceSensor.get_distance();

//         if (dist <= 475) break;

//         chassis.arcade(-90, 0);
//         pros::delay(10);
//     }
// chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
// chassis.arcade(0,0);

float targetHeading = chassis.getPose().theta;  // save starting angle

    while (true) {
        int dist = distanceSensor.get_distance();
        if (dist <= 550 && dist > 0) break;


        int errorDist = dist - 150;
        int forward = errorDist * 0.3;

        // clamp speed so it doesn't go crazy
        if (forward > 90) forward = 90;
        if (forward < 0) forward = 0;  // minimum so robot keeps moving


        float currentHeading = chassis.getPose().theta;

        // error = how far we drifted
        float error = targetHeading - currentHeading;

        // simple P correction (THIS is the "PID")
        float kP = 4.0;   // tune this later
        float turn = error * kP;

        // clamp so it doesn’t oversteer
        if (turn > 50) turn = 50;
        if (turn < -50) turn = -50;

        chassis.arcade(-forward, turn);

        pros::delay(10);
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.arcade(0, 0);





}

void driveWithDistanceHoldHeading(pros::Distance& sensor,
                                  int stopDistance,
                                  bool stopWhenGreaterOrEqual,
                                  bool reverseDrive,
                                  int forwardOffset = 0,
                                  float forwardScale = 1.0f,
                                  float turnKp = 4.0f) {
    const float targetHeading = chassis.getPose().theta;

    while (true) {
        const int dist = sensor.get_distance();
        const bool reachedTarget =
            stopWhenGreaterOrEqual ? (dist >= stopDistance) : (dist <= stopDistance);
        if (dist > 0 && reachedTarget) break;

        int forward = static_cast<int>((dist - 150) * forwardScale) + forwardOffset;

        if (forward > 90) forward = 90;
        if (forward < 0) forward = 0;

        const float currentHeading = chassis.getPose().theta;
        const float error = targetHeading - currentHeading;

        float turn = error * turnKp;
        if (turn > 50) turn = 50;
        if (turn < -50) turn = -50;

        chassis.arcade(reverseDrive ? -forward : forward, turn);
        pros::delay(10);
    }

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.arcade(0, 0);
}

void states_auto(){
    chassis.setPose(0,0,0);

    intakeBlocks();

    // first forward
    driveWithDistanceHoldHeading(distanceSensor, 585, true, false);

    // turn 90 deg
    chassis.turnToHeading(90, 750);
    pros::delay(1000);

    driveWithDistanceHoldHeading(distanceSensorfront, 650, false, false);

    chassis.turnToHeading(180,500);
    pros::delay(1000);

    driveWithDistanceHoldHeading(distanceSensorfront, 200, false, false);

    chassis.turnToHeading(180,500);
    pros::delay(1000);

    driveWithDistanceHoldHeading(distanceSensorfront, 500, true, true, 50);

    chassis.turnToHeading(90,500);
    pros::delay(1000);

    driveWithDistanceHoldHeading(distanceSensorfront, 150, false, false, 30);

    chassis.turnToHeading(0, 750);
}

void skills_full_auton_75(){ 


    int second_scoring_x = -64.;
    int third_and_fourth_scraper_x = -64;

    //x= 1 , y= 0.40
    //x= 1, y = 1 


    intakeBlocks();

    //rightlong();
    //rightlongtomiddle();
    // middletoleftlong();

    //sevenballright();

    //skillsfirstgoal();
    // Start Position 
    piston.set_value(false);
    chassis.setPose(0,0,0); 

    // heading = 170 , 1.15 and y = 0.13
 

    // Move to Loader 1 (Bottom Right)
    chassis.moveToPoint(0, 10,750);
    chassis.turnToHeading(90,750);

    pros::delay(1000);
    int distance_for_goal1 = (0.95 - gps.get_position().y ) * 39.37 ;

    chassis.moveToPoint(distance_for_goal1,10, 1000);

    int current_position = gps.get_position().y;
    if (current_position < 1.01)
    {    
        distance_for_goal1 = (1.04 - current_position) * 39.37 ;
        int current_pos = chassis.getPose().x;
        chassis.moveToPoint(current_pos + distance_for_goal1, 10, 100);
    }

    //distance_for_goal1 = (1.01 - gps.get_position().y ) * 39.37 ;
    

    //chassis.moveToPoint(distance_for_goal1,10, 1000);


    //fixPosition(500,1);
    scraperDown();
    chassis.turnToHeading(180,750);




    //Loader 1 Matchloading or Scraping (Bottom Right)
    chassis.moveToPoint(31, -17,1500, {.maxSpeed=70}); // On Loader 1  

    chassis.moveToPoint(31, 0, 500, {.forwards=false});

    chassis.moveToPoint(31, -17,1000, {.maxSpeed=70}); // On Loader 1  

    //chassis.moveToPoint(30, 0, 1500, {.forwards=false});




    // //Entering Loader 1 again (Bottom right)
    // chassis.moveToPoint(30, 0, 750, {.forwards=false});

    // chassis.moveToPoint(30, -15,2500); // On Loader 1 
    // Begin the movement to the other side
    chassis.moveToPoint(30, 0, 1500, {.forwards=false});

    scraperUp();
    chassis.turnToHeading(45,750);
    piston.set_value(true);

    // Turning and moving to other side (Lower right side of goal)
    chassis.moveToPose(42, 29, 0, 2000);

    chassis.moveToPose(42, 100, 0, 1500, {.maxSpeed=80}); // earlier value 42 
    //chassis.turnToHeading(275, 750);
    chassis.turnToHeading(315,1000);
    chassis.moveToPoint(30, 105, 1000);
    // pros::delay(750);

    // float targetHeading1 = chassis.getPose().theta;  // save starting angle

    // while (true){
    //     int dist = distanceSensor.get_distance();
    //     if (dist >= 300 && dist > 0) break;


    //     int errorDist = dist - 150;
    //     int forward = errorDist * 0.3;

    //     // clamp speed so it doesn't go crazy
    //     // if (forward > 90) forward = 90;
    //     // if (forward < 20) forward = 20;  // minimum so robot keeps moving


    //     float currentHeading = chassis.getPose().theta;

    //     // error = how far we drifted
    //     float error = targetHeading1 - currentHeading;

    //     // simple P correction (THIS is the "PID")
    //     float kP = 2.0;   // tune this later
    //     float turn = error * kP;

    //     // clamp so it doesn’t oversteer
    //     if (turn > 50) turn = 50;
    //     if (turn < -50) turn = -50;

    //     chassis.arcade(70, turn);

    //     pros::delay(10);

    // }
    // chassis.arcade(0, 0);

    // pros::delay(750);
    // chassis.setPose(31,91,-90);
    

    chassis.turnToHeading(0,750);

    // Scoring the long goal
    chassis.moveToPose(30, 50, 0, 3000,{.forwards=false,.maxSpeed=100});
    pros::delay(750);


    goalHighScoring();
    pros::delay(2500);
    piston.set_value(false);


    intakeBlocks();
   

    // Matchload or scrape Loader 2 (Top Right Side)
    scraperDown();
    chassis.moveToPoint(30, 130, 1500, {.maxSpeed=70});
    chassis.moveToPoint(30, 100, 500, {.forwards=false,.maxSpeed=60});
    chassis.moveToPoint(30, 130, 2000, {.maxSpeed=50});


    // chassis.turnToHeading(10,300);
    // chassis.turnToHeading(350,300);00ok
    // chassis.moveToPoint(30, 130, 500, {.maxSpeed=70});

    // chassis.turnToHeading(bn10,300);
    // chassis.turnToHeading(350,300);

    




    // Score again
    chassis.moveToPoint(31, 83, 1500,{.forwards=false,.minSpeed=50});
    pros::delay(750);
    goalHighScoring();

    pros::delay(1000); 
    
    topintake.move_velocity(600); 
    lowIntake.move_velocity(600);
    //scraperDown();
    pros::delay(300);

    goalHighScoring();
    pros::delay(2000);
    

  

//     scraperUp();    
//     // Move to left side
    chassis.moveToPoint(29, 95, 750);





    chassis.turnToHeading(90,750);

    pros::delay(750);
    float targetHeading = chassis.getPose().theta;  // save starting angle

    while (true) {
        int dist = distanceSensor.get_distance();
        if (dist <= 600 && dist > 0) break;


        int errorDist = dist - 150;
        int forward = errorDist * 0.3;

        // clamp speed so it doesn't go crazy
        if (forward > 90) forward = 90;
        if (forward < 0) forward = 0;  // minimum so robot keeps moving


        float currentHeading = chassis.getPose().theta;

        // error = how far we drifted
        float error = targetHeading - currentHeading;

        // simple P correction (THIS is the "PID")
        float kP = 2.0;   // tune this later
        float turn = error * kP;

        // clamp so it doesn’t oversteer
        if (turn > 50) turn = 50;
        if (turn < -50) turn = -50;

        chassis.arcade(-forward, turn);

        pros::delay(10);
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.arcade(0, 0);


    pros::delay(500);




    //chassis.moveToPoint(third_and_fourth_scraper_x, 94, 2000);
    ////fixPosition_neg_x(500, 1);
    chassis.turnToHeading(0,750);
    intakeBlocks();

    scraperDown();
    pros::delay(1000); 


    // 3rd Scraper
    chassis.moveToPoint(third_and_fourth_scraper_x , 130, 2000,{.maxSpeed=70});

//     // Moving to other side
    chassis.moveToPoint(third_and_fourth_scraper_x , 95, 750,{.forwards=false});

    chassis.moveToPoint(third_and_fourth_scraper_x, 130, 2000,{.maxSpeed=60});

    chassis.moveToPoint(third_and_fourth_scraper_x, 95, 800,{.forwards=false});
    pros::delay(750);
        
    piston.set_value(true);

    chassis.turnToHeading(225,750);
    chassis.moveToPose(-78, 72, 180,2000);
    chassis.moveToPose(-77, 8, 180,2000);


//     /// Prepare to Score 

    chassis.turnToHeading(135,750);
    chassis.moveToPoint(third_and_fourth_scraper_x, 0, 1000);

    pros::delay(750);

    //float targetHeading2 = chassis.getPose().theta;  // save starting angle

    // while (true){
    //     int dist = distanceSensor.get_distance();
    //     if (dist >= 300 && dist > 0) break;


    //     int errorDist = dist - 150;
    //     int forward = errorDist * 0.3;

    //     // clamp speed so it doesn't go crazy
    //     // if (forward > 90) forward = 90;
    //     // if (forward < 20) forward = 20;  // minimum so robot keeps moving


    //     float currentHeading = chassis.getPose().theta;

    //     // error = how far we drifted
    //     float error = targetHeading2 - currentHeading;

    //     // simple P correction (THIS is the "PID")
    //     float kP = 2.0;   // tune this later
    //     float turn = error * kP;

    //     // clamp so it doesn’t oversteer
    //     if (turn > 50) turn = 50;
    //     if (turn < -50) turn = -50;

    //     chassis.arcade(70, turn);

    //     pros::delay(10);

    // }
    // chassis.arcade(0, 0);

    // return;


    chassis.turnToHeading(180,1000, {.maxSpeed=60});


//     //Second  Score 

    chassis.moveToPoint(second_scoring_x, 30, 2000,{.forwards=false});
    pros::delay(750);
    piston.set_value(false);

    goalHighScoring();

    pros::delay(1000); 

    
    topintake.move_velocity(600); 
    lowIntake.move_velocity(600);
    //scraperDown();
    pros::delay(300);

    goalHighScoring();
    pros::delay(1500);
    

//     scraperDown();
//     pros::delay(1000);


//     // 4th Scraper


    chassis.moveToPoint(third_and_fourth_scraper_x+1 , -30, 1500,{.maxSpeed= 70});

    chassis.moveToPoint(second_scoring_x, 0, 500,{.forwards=false});

    chassis.moveToPoint(third_and_fourth_scraper_x+1 , -30, 2000,{.maxSpeed= 70});

    intakeBlocks();

    // Score again 

    chassis.moveToPoint(second_scoring_x, 30, 2000,{.forwards=false});
    pros::delay(750);

    goalHighScoring();

    pros::delay(1000); 


    goalHighScoring();
    pros::delay(1500);

    
    
    


    piston.set_value(true);

    // Parking 
    chassis.moveToPoint(-62, 8, 750, {.maxSpeed=1000});
    chassis.turnToHeading(90,750, {.maxSpeed=1000});
    chassis.moveToPoint(-16, 8, 1000, {.maxSpeed=1000});
    chassis.turnToHeading(180,750, {.maxSpeed=1000});
    lowIntake.move_velocity(600);
    topintake.move_velocity((600));
    //chassis.moveToPoint(-16, 20, 1000, {.forwards=false, .maxSpeed=1000});
    chassis.moveToPoint(-16, -20000 , 20000, {.maxSpeed=1000});












    

    

//     //printCoordinates();

// }
// void sevenballRight_NEW(){
//     piston.set_value(false);

//     chassis.setPose(0,0,0); 
//     intakeBlocks();
//     // Go to Right Lower Loader
//     chassis.moveToPoint(0, 10,500,{.maxSpeed=100});
//     chassis.turnToHeading(30,500,{.maxSpeed=60});
//     chassis.moveToPoint(8,25, 750,{.maxSpeed=100});
//     pros::delay(500);
//     scraperDown();
//     chassis.turnToHeading(135,500,{.maxSpeed=60});
//     chassis.moveToPoint(36,9, 750,{.maxSpeed=100});
//     chassis.turnToHeading(180,500,{.maxSpeed=60});
//     //chassis.moveToPoint(32,-30, 1250,{.maxSpeed=100}); // Scraping


//     chassis.moveToPoint(31,24, 1250,{.forwards=false,.maxSpeed=100}); // Scoring 
//     pros::delay(500);
//     scraperUp();
//     goalHighScoring();
//     pros::delay(1000);

//     chassis.moveToPoint(31,8, 750,{.maxSpeed=100});
//     chassis.turnToHeading(125,500,{.maxSpeed=60});

//     chassis.moveToPoint(20,14, 750,{.forwards=false,.maxSpeed=100});
//     chassis.turnToHeading(180,500,{.maxSpeed=60});
//     piston.set_value(true);
//     chassis.moveToPoint(19,38, 1500,{.forwards=false,.maxSpeed=100}); //   

}

void rightSideDoubleGoal_NEW(){

    piston.set_value(true);

    chassis.setPose(0,0,0); 
    intakeBlocks();
    // Go to Right Lower Loader
    chassis.moveToPoint(0, 10,500,{.maxSpeed=100});
    chassis.turnToHeading(27,500,{.maxSpeed=60});
    chassis.moveToPoint(7,26, 1250,{.maxSpeed=100});
    pros::delay(500);
    piston.set_value(false);

    chassis.turnToHeading(315,400,{.maxSpeed=60});
    piston.set_value(true);
    chassis.moveToPoint(-4,40, 1000,{.maxSpeed=100});// Low Scoring
    lowgoalscoring();
    pros::delay(1000);

   
    chassis.moveToPoint(4,29, 750,{.forwards=false,.maxSpeed=100});
    pros::delay(500);

    chassis.turnToHeading(135,500,{.maxSpeed=60});
    chassis.moveToPoint(30,9, 1250,{.maxSpeed=100});
    piston.set_value(false);
    chassis.turnToHeading(180,500,{.maxSpeed=60});
    chassis.moveToPoint(30,-40, 1500,{.maxSpeed=60}); // Scraping


    chassis.moveToPoint(30,24, 1500,{.forwards=false,.maxSpeed=100}); // Scoring 
    pros::delay(500);
    goalHighScoring();
    pros::delay(2000);
    chassis.moveToPoint(30,8, 1000,{.maxSpeed=100});
    chassis.turnToHeading(125,500,{.maxSpeed=60});

    chassis.moveToPoint(20,14, 500,{.forwards=false,.maxSpeed=100});
    chassis.turnToHeading(180,500,{.maxSpeed=60});
    //piston.set_value(true);
    chassis.moveToPoint(19,38, 1500,{.forwards=false,.maxSpeed=100}); //   


    

}
void solopp(){
    //goalHighScoring(); // Should be commented out later..
    //rightlong();
    //rightlongtomiddle();
    // middleto leftlong();
    //sevenballright();
    //skillsfirstgoal();
    // Start Position 
    chassis.setPose(0,0,0); 

    // Go to Right Lower Loader
    chassis.moveToPoint(0, 10,500,{.maxSpeed=100});
    chassis.turnToHeading(90,500,{.maxSpeed=60});

    chassis.moveToPoint(30,10, 750,{.maxSpeed=100});
    //fixPosition(500,1);
    chassis.turnToHeading(180,500,{.maxSpeed=60});

    scraperDown();
    intakeBlocks();

    //Right Lower Loader Scraping
    chassis.moveToPoint(30, -16,2000); // On Loader 1 

    // reverse to score first set 
    chassis.moveToPoint(30, 25, 2000, {.forwards=false,.maxSpeed=75});
    scraperUp();
    pros::delay(500);

    goalHighScoring();
    pros::delay(750);

    return;

    //moving forward afer scoring
    chassis.moveToPoint(30, 11,400,{.maxSpeed=75}); 
    //turn to get sencond set
    chassis.turnToHeading(315,750,{.maxSpeed=75});
    intakeBlocks();
    //get to score positison
    chassis.moveToPoint(10, 21,750,{.maxSpeed=75});

    pros::delay(500);
    scraperDown();
    pros::delay(1000);
    chassis.turnToHeading(315,400);



    scraperUp();
    chassis.moveToPoint(-3, 26,3000,{.maxSpeed=75}); 
    lowgoalscoring();
    pros::delay(500);


    chassis.moveToPoint(4.7, 17.6,3000,{.forwards=false}); 



    // chassis.turnToHeading(270,400,{.maxSpeed=80});
    // intakeBlocks();

    // chassis.moveToPoint(-37, 25.5,1500,{.maxSpeed=80}); 
    // pros::delay(750);
    // scraperDown();

    // chassis.turnToHeading(240, 500,{.maxSpeed=80});

    // chassis.moveToPoint(-64, 5,800,{.maxSpeed=80});

    // chassis.turnToHeading(180,500,{.maxSpeed=80});

    // chassis.moveToPoint(-64, -15,1000,{.maxSpeed=80});

    // // chassis.turnToHeading(180,750);

    // chassis.moveToPoint(-64, 30,2000,{.forwards=false}); 
    // pros::delay(500);

    // // chassis.moveToPoint(-64, 14,2000);// Scoring
    // goalHighScoring();

    // printCoordinates();

    //printCoordinates();



}

void ruiguansoloopp(){

    chassis.setPose(0,0,0);

    intakeBlocks();

    chassis.moveToPoint(0, 39, 750);
    chassis.turnToHeading(90,750,{.maxSpeed=80});
    pros::delay(100);

    piston.set_value(false);

    chassis.moveToPoint(25, 34, 1250,{.maxSpeed=60});

    chassis.moveToPoint(-20, 33, 1000, {.forwards=false, .minSpeed=70});

    piston.set_value(true);

    pros::delay(1000);

    goalHighScoring();

    pros::delay(1000);

    intakeBlocks();
    chassis.moveToPoint(-8, 34, 500);
    chassis.turnToHeading(212, 500);

    chassis.moveToPoint(-24, 6,1000);

    chassis.turnToHeading(180, 500);

    chassis.moveToPoint(-20, -44, 1000);

    chassis.turnToHeading(134,500);

    chassis.moveToPoint(-35, -24, 1000,{.forwards=false});
    pros::delay(250);

    piston3.set_value(true);
    pros::delay(1000);

    goalHighScoring();
    piston3.set_value(false);
    intakeBlocks();


    chassis.moveToPoint(0, -63, 1000);

    chassis.turnToHeading(90, 750);

    piston.set_value(false);


    chassis.moveToPoint(50, -56,1250,{.minSpeed=70});

    chassis.moveToPoint(-20, -60,1000,{.forwards=false});
    pros::delay(500);

    goalHighScoring();





}





void autonomous() {
    states_auto();


    //distancesensortest();
    //fourballrightrush();

    


    //skills_full_auton_75();
    //ruiguansoloopp();
    //solopp();
    //sevenballRight_NEW(); // Working
    //rightSideDoubleGoal_NEW();

    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0, 50, 5000);



    //sevenballright();

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

    //solopp();
    //skills_full_auton_75();
    //intakeBlocks();
    //goalHighScoring();



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

        controller.print(0,0, "X: %f, Y: %f", chassis.getPose().x,chassis.getPose().y);// x
        //printf(1, "Y: %f", chassis.getPose().y); // y
        //printf(2, "Theta: %f", chassis.getPose().theta); // heading
        
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);

        //pros::ADIDigitalOut piston('G'); // scrapper
        //pros::ADIDigitalOut piston1('H'); // decore wing
        //pros::ADIDigitalOut piston2('C'); //middle goal descore
        //pros::ADIDigitalOut piston3('B'); //middle goal dropper
		


		if (controller.get_digital(DIGITAL_L2)) {

            lowIntake.move_velocity(-600);

                
            
        }
        else if (controller.get_digital(DIGITAL_DOWN)){

            lowIntake.move_velocity(600);

        }
        else{

            lowIntake.move_velocity(0);


        }

        if (controller.get_digital(DIGITAL_R1)) {
            piston1.set_value(true);
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            piston1.set_value(false);
        }

        if (controller.get_digital(DIGITAL_X)) {
            piston.set_value(true);
        }
        else if (controller.get_digital(DIGITAL_B)) {
            piston.set_value(false);
        }

        if (controller.get_digital(DIGITAL_UP)) {
            piston3.set_value(true);
        }
        else if (controller.get_digital(DIGITAL_DOWN)) {
            piston3.set_value(false);
        }

        if (controller.get_digital(DIGITAL_RIGHT)) {
            piston2.set_value(true);
        }
        else if (controller.get_digital(DIGITAL_LEFT)) {
            piston2.set_value(false);
        }

		if (controller.get_digital(DIGITAL_L1)) {

			topintake.move_velocity(-600);   
            indexer.move_velocity(600);

		}
        else{

            topintake.move_velocity(0);  
            indexer.move_velocity(0);
        }


        
        
    

		
		pros::delay(2);

    }

}

	