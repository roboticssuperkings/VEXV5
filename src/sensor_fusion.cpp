#include "main.h"
#include <cmath>

// these are the sensors we're using (they're set up in main.cpp)
extern pros::GPS gps;
extern pros::Distance distanceSensor;
extern pros::Distance distanceSensorfront;
extern lemlib::Chassis chassis;

// this function moves the robot to a spot and double checks if it actually got there
// targetX and targetY = where we want to go (in inches)
// heading = which way the robot should face at the end (in degrees)
// timeout = how long to try before giving up (in milliseconds)
void goToPosition(float targetX, float targetY, float heading, int timeout = 2000) {

    // first just try to go there normally
    chassis.moveToPoint(targetX, targetY, timeout);
    pros::delay(100); // wait a bit so sensors can update

    // now lets see where the robot thinks it is
    float odomX = chassis.getPose().x;
    float odomY = chassis.getPose().y;

    // also check GPS (it gives meters so we multiply by 39.37 to get inches)
    float gpsX = gps.get_position().x * 39.37;
    float gpsY = gps.get_position().y * 39.37;

    // get distance sensor reading too (we might need it later)
    float distReading = distanceSensor.get_distance();

    // figure out how far off each sensor thinks we are from the target
    // this is just the distance formula from math class lol
    float odomErr = sqrt(pow(odomX - targetX, 2) + pow(odomY - targetY, 2));
    float gpsErr = sqrt(pow(gpsX - targetX, 2) + pow(gpsY - targetY, 2));

    // pick which sensor to trust
    // basically whoever says we're closest to the target is probably right
    float correctedX, correctedY;

    if (gpsErr < odomErr && gpsErr < 10) {
        // gps seems more accurate, use that
        correctedX = gpsX;
        correctedY = gpsY;
    } else {
        // odometry seems better or gps is being weird
        correctedX = odomX;
        correctedY = odomY;
    }

    // check if we're actually close enough to where we wanted to go
    float positionError = sqrt(pow(correctedX - targetX, 2) +
                               pow(correctedY - targetY, 2));

    if (positionError > 2.0) {
        // we're more than 2 inches off, so fix it
        // tell the robot where it actually is
        chassis.setPose(correctedX, correctedY, chassis.getPose().theta);

        // now try to get to the exact spot (go slow so we don't overshoot)
        chassis.moveToPoint(targetX, targetY, 500, {.maxSpeed = 60});
    }

    // finally turn to face the right direction
    chassis.turnToHeading(heading, 750);
}

// same thing but also uses the distance sensor to check how far from a wall we are
// use this when the robot is facing straight at a wall
// expectedWallDist = how far we should be from the wall (in inches)
void goToPositionWithDistanceCheck(float targetX, float targetY, float heading,
                                    float expectedWallDist, int timeout = 2000) {

    // go to the spot first
    chassis.moveToPoint(targetX, targetY, timeout);
    pros::delay(100);

    // get all the sensor readings
    float odomX = chassis.getPose().x;
    float odomY = chassis.getPose().y;
    float gpsX = gps.get_position().x * 39.37;
    float gpsY = gps.get_position().y * 39.37;
    float distReading = distanceSensor.get_distance() / 25.4; // convert mm to inches

    // see how wrong each sensor is
    float odomErr = sqrt(pow(odomX - targetX, 2) + pow(odomY - targetY, 2));
    float gpsErr = sqrt(pow(gpsX - targetX, 2) + pow(gpsY - targetY, 2));
    float distErr = fabs(distReading - expectedWallDist);

    // start by trusting odometry
    float correctedX = odomX;
    float correctedY = odomY;
    float minErr = odomErr;

    // if gps is better, use that instead
    if (gpsErr < minErr && gpsErr < 10) {
        minErr = gpsErr;
        correctedX = gpsX;
        correctedY = gpsY;
    }

    // check if we're facing a wall straight on (0, 90, 180, or 270 degrees)
    float currentHeading = chassis.getPose().theta;
    bool headingAligned = (fabs(fmod(currentHeading, 90)) < 5);

    // if distance sensor is the most accurate AND we're facing a wall
    if (distErr < minErr && headingAligned && distReading > 0 && distReading < 60) {
        // use distance sensor to figure out our position
        // depends on which way we're facing
        if (fabs(currentHeading - 180) < 10) {
            // facing towards bottom of field
            correctedY = distReading;
        } else if (fabs(currentHeading - 0) < 10) {
            // facing towards top of field
            correctedY = 144 - distReading; // field is 144 inches wide
        } else if (fabs(currentHeading - 90) < 10) {
            // facing right
            correctedX = 144 - distReading;
        } else if (fabs(currentHeading - 270) < 10) {
            // facing left
            correctedX = distReading;
        }
    }

    // if we're off by more than 2 inches, fix it
    float positionError = sqrt(pow(correctedX - targetX, 2) +
                               pow(correctedY - targetY, 2));

    if (positionError > 2.0) {
        chassis.setPose(correctedX, correctedY, chassis.getPose().theta);
        chassis.moveToPoint(targetX, targetY, 500, {.maxSpeed = 60});
    }

    // turn to face the right way
    chassis.turnToHeading(heading, 750);
}
