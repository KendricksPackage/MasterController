#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
//#include "pros/misc.h"
//#include <utility>
using namespace pros;
Controller master(E_CONTROLLER_MASTER);

pros::Motor firstIntake(13);
pros::Motor secondIntake(1);
pros::Motor LeftArmMotor(9);
pros::Motor RightArmMotor(-11);
pros::adi::DigitalOut hang(2);
pros::adi::DigitalOut clamp(1);

//Motor group
pros::MotorGroup leftMotor({-7,-2},pros::MotorGears::blue);
pros::MotorGroup rightMotor({15,6},pros::MotorGears::blue);
pros::MotorGroup arm({9,-11},pros::v5::MotorGears::red);

pros::v5::Distance distance(4);

lemlib::Drivetrain drivetrain(
							&leftMotor, // left motor group
							&rightMotor, // right motor group
							12.75, // 12.75 inch track width
							lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							450, // drivetrain rpm is 360
							8 // chase power is 2. If we had traction wheels, it would have been 8
);

// forward/backward PID
lemlib::ControllerSettings lateral_controller(
							4, // proportional gain (kP)
							0, // integral gain (kI)
							10, // derivative gain (kD)
							3, // anti windup
							1, // small error range, in inches
							100, // small error range timeout, in milliseconds
							3, // large error range, in inches
							500, // large error range timeout, in milliseconds
							15 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                              0.5, // integral gain (kI)
                                              17, // derivative gain (kD) [ORIGNAL IS 17]
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// uses "vert" as the encoder. Old 2.75" wheel, 4.3" left of the tracking center, 2:1 gear ratio

pros::Rotation vert(-8);


//lemlib::TrackingWheel left_tracking_wheel(&left_enc, lemlib::Omniwheel::NEW_275, -4.3, 2);
lemlib::TrackingWheel trackwheel(&vert, lemlib::Omniwheel::NEW_275,0.5);

pros::Imu imu(10);

lemlib::OdomSensors sensors(&trackwheel, //  tracking wheel 1
							nullptr,
							nullptr,
							nullptr,
                             //  tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	pros::lcd::initialize();
	pros::Task screen_task([&]() {
	while (true) { // infinite loop
        // print measurements from the adi encoder
  		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::delay(20); // delay to save resources. DO NOT REMOVE
    }
	});
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


// ----------------------------- Pure Pursuit  --------------------------------- // 
//redRush
ASSET(redSwipe_txt);
ASSET(redStraight_txt);
ASSET(redSwipe1_txt);


//blueRush 
ASSET(blueSwipe_txt);
ASSET(blueStraight_txt);

//skills
ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);

// ----------------------------- Auto Functions --------------------------------- // 

// RED SIDE

void redRush(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -27.37, 0, 2000,{.forwards = false, .minSpeed = 105, .earlyExitRange = 4});
	// chassis.moveToPose(10.5, -45.5, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.moveToPose(10.5, -45, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(100);
	clamp.set_value(1);
	//pros::delay(200);
	chassis.turnToHeading(20, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(10.5, -30.5, 11, 1500,{.minSpeed = 127});
	chassis.moveToPose(10.3, 0, 11, 2000,{.minSpeed = 127});
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntil(15);
	firstIntake.move(127);
	secondIntake.move(127);
	delay(200);
	chassis.moveToPose(10.3, -25, 11, 2000,{.forwards = false, .minSpeed = 50});
	delay(600);
	firstIntake.move(0);
	secondIntake.move(0); // only for testing purposes
	chassis.waitUntilDone();
	//chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 300);
	//chassis.waitUntilDone();
	clamp.set_value(0);
	chassis.turnToHeading(275, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(51, -25, 275, 1000,{.forwards = false, .minSpeed = 40});
	chassis.waitUntilDone();
	clamp.set_value(1);
	arm.move_absolute(3000, 127);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.turnToHeading(50, 300);
	chassis.moveToPose(63, 5, 50, 1800);
	chassis.moveToPose(51, -15, 0, 1000,{.forwards = false, .minSpeed = 127});
	chassis.waitUntilDone();
	arm.move_absolute(-1, 80);
	delay(1200);
	clamp.set_value(0);
	delay(500);
	chassis.moveToPose(55,-30,180,1300);
	firstIntake.move(0);
	//secondIntake.move(0);

}

void redRushTIMEWASTE(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -27.37, 0, 2000,{.forwards = false, .minSpeed = 105, .earlyExitRange = 4});
	// chassis.moveToPose(10.5, -45.5, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.moveToPose(10.5, -45, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(300);
	clamp.set_value(1);
	pros::delay(200);
	chassis.turnToHeading(20, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(10.5, -30.5, 11, 1500,{.minSpeed = 127});
	chassis.moveToPose(10.3, 0, 11, 2000,{.minSpeed = 127});
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntil(15);
	firstIntake.move(127);
	secondIntake.move(127);
	delay(400);
	chassis.moveToPose(10.3, -25, 11, 2000,{.forwards = false, .minSpeed = 50});
	chassis.waitUntilDone();
	//chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 500);
	chassis.waitUntilDone();
	firstIntake.move(0);
	secondIntake.move(0); // only for testing purposes
	clamp.set_value(0);
	chassis.turnToHeading(275, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(51, -25, 275, 1000,{.forwards = false, .minSpeed = 40});
	chassis.waitUntilDone();
	clamp.set_value(1);
	chassis.turnToHeading(50, 300);
	chassis.moveToPose(65, 5, 50, 1800);
	chassis.waitUntilDone();
	firstIntake.move(127);
	//delay(150);
	chassis.moveToPose(60,-6.5,40,700,{.forwards = false, .minSpeed = 40});
	chassis.turnToHeading(-40, 400);
	firstIntake.move(-127);
	secondIntake.move(-127);
	chassis.waitUntilDone();
	chassis.turnToHeading(50, 300);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(66,1.5,35,800,{.minSpeed = 40});
	chassis.waitUntilDone();
	chassis.turnToHeading(0,200);
	chassis.moveToPose(65, 16,0,1000,{.minSpeed = 127});
	chassis.turnToHeading(270, 350);
	delay(600);
	firstIntake.move(0);
	secondIntake.move(0);
	clamp.set_value(0);
	chassis.waitUntilDone();
	chassis.moveToPose(60,-35,180,1300);
	arm.move_absolute(3000, 127);


}

void redRushITWORKS(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -27.37, 0, 2000,{.forwards = false, .minSpeed = 105, .earlyExitRange = 4});
	// chassis.moveToPose(10.5, -45.5, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.moveToPose(10.5, -45, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(300);
	clamp.set_value(1);
	pros::delay(200);
	chassis.turnToHeading(20, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(10.5, -30.5, 11, 1500,{.minSpeed = 127});
	chassis.moveToPose(10.3, 0, 11, 2000,{.minSpeed = 127});
	//chassis.turnToHeading(80, 500);
	//chassis.moveToPose(57, 3, 59, 3000);
	//chassis.waitUntilDone();
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntil(15);
	firstIntake.move(127);
	secondIntake.move(117);
	delay(800);
	chassis.moveToPose(10.3, -15, 11, 2000,{.forwards = false, .minSpeed = 50});
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.turnToHeading(85, 500);
	firstIntake.move(0);
	//chassis.moveToPose(8, -8, 80,3000);
	chassis.moveToPose(65, 0,85 , 1500);
	chassis.waitUntilDone();
	firstIntake.move(127);
	chassis.moveToPose(50, 0, 90, 1500,{.forwards = false});
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.turnToHeading(0, 500);
	firstIntake.move(-127);
	secondIntake.move(-127);
	chassis.turnToHeading(90, 500);
	chassis.moveToPose(70,0,90,1500);
	firstIntake.move(127);
	secondIntake.move(117);
	chassis.turnToHeading(0, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(90,5,0,1000,{.forwards = false});
	chassis.waitUntilDone();
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	clamp.set_value(0);
	arm.move_absolute(-6480, 127);
	// chassis.moveToPose(59, -20, 59, 1500);
	// chassis.turnToHeading(140, 800);
	// firstIntake.move(-127);
	// secondIntake.move(-127);
	// chassis.turnToHeading(59, 800);
	// chassis.moveToPose(55, 10, 59, 1000);
	// firstIntake.move(0);
	// secondIntake.move(0);
}

void redRush3(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(-0.3107,-23.373,0,1500,{.forwards = false, .minSpeed = 127});
	chassis.moveToPose(12, -45, -27.88, 1300, {.forwards = false, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(500);
	clamp.set_value(1);
	pros::delay(500);
}
void redRush2(){
	chassis.setPose(0,0,0);
	//chassis.moveToPose(-0.28, -22.1447, 0.75, 1000,{.forwards = false, .minSpeed = 127});
	// chassis.moveToPose(0.565, -26.37, -25.208, 1000,{.forwards = false});
	chassis.moveToPose(7.72, -33.64, -25.5, 1500,{.forwards = false, .minSpeed = 127});
	chassis.moveToPose(8.29, -42.5, -27.6598, 1000,{.forwards = false}); // original y = -40.999
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	clamp.set_value(1);
	pros::delay(500);
	chassis.moveToPose(9.24, -39.98, 11.11, 10000);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(10.97, -31.365, 11.94, 1000);
	chassis.moveToPose(12.43, -31.279, 68.98, 1300);
	pros::delay(1500);
	firstIntake.move(0);
	secondIntake.move(0);
}
void redRush1(){
	chassis.setPose(-58.486,-58.653,280);
	//chassis.moveToPoint(-15.7, -55.874, 1500,{.forwards = false});
	//chassis.follow(redSwipe1_txt,10,2500,false);
	chassis.moveToPose(-16.3,-56.9,255,1850,{.forwards = false, .maxSpeed = 65});
	//  chassis.moveToPose(-73.60,-55.98,280,1500,{.forwards = false, .maxSpeed = 70});
	//chassis.waitUntilDone();
	// chassis.moveToPose(-15.58, -56.9, 250, 1500,{.forwards = false, .maxSpeed = 70});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	clamp.set_value(1);
	pros::delay(500);
	chassis.turnToHeading(295,1000);
	chassis.waitUntilDone();
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(-23,-51.449,295,1500,{true});
	chassis.waitUntilDone();
	chassis.turnToHeading(30, 1000);
	chassis.moveToPose(-46.951,0,360,2500);
	chassis.waitUntilDone();
	chassis.moveToPoint(-46.951, 5, 900);
	secondIntake.move(0);
	chassis.moveToPose(-46.951,-4,350,800,{.forwards = false});
	chassis.waitUntilDone();
	chassis.turnToHeading(290,1000);
	firstIntake.move(-127);
	secondIntake.move(-127);
	pros::delay(600);
	firstIntake.move(0);
	secondIntake.move(0);
	chassis.turnToHeading(350,1000);
	chassis.moveToPose(-48.951,30,345,2500);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(-5, -25, 140, 1500);
	chassis.waitUntilDone();

}
void redFar(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0,-25, 0,1200,{.forwards = false, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(100);
	clamp.set_value(1);
	//pros::delay(300);
	//chassis.moveToPose(5.4, -25.377, 76, 1000);
	//chassis.turnToHeading(55,300);
	chassis.turnToHeading(72,400);
	firstIntake.move(127);
	secondIntake.move(127);
	//chassis.moveToPose(20, -1.5, 55, 1300);
	chassis.moveToPose(22, -15, 60, 1200); // get first on field
	chassis.turnToHeading(140, 400);
	chassis.moveToPose(14.3,-20, 130,1000);
	//chassis.moveToPose(20, -30, 156,1300);
	chassis.waitUntilDone();
	chassis.moveToPose(25, -39, 155, 900);
	//chassis.turnToHeading(185, 300);
	chassis.moveToPose(18, -25, 155, 1000,{.forwards = false});
	chassis.turnToHeading(110, 400);
	chassis.moveToPose(33, -31, 110, 1300);
	chassis.moveToPose(15, -30, 110, 1000,{.forwards = false});
	delay(900);
	//primary
	chassis.turnToHeading(-84, 500);
	//arm.move_absolute(3150,127); for ring on red wall goal 
	//chassis.moveToPose(-20, -21, -83, 2600);
	chassis.moveToPose(-12,-38,130,1400,{.maxSpeed = 70});
	chassis.waitUntil(10);
	clamp.set_value(0);
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);


	/* get the ring on the red wall goal 
	chassis.moveToPose(-25,-17,240,3000);
	chassis.waitUntil(20);
	clamp.set_value(0);
	chassis.moveToPose(-28.5, -20.5, 240,900,{.forwards = false});
	arm.move_absolute(-20, 127);
	while(distance.get() >= 35){
		secondIntake.move(90);
	}
	firstIntake.move(-90);
	secondIntake.move(-90);
	delay(500);
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -10, 0,1200,{.forwards = false, .minSpeed = 60});


	firstIntake.move(-127);
	secondIntake.move(0);
	chassis.waitUntilDone();
	


	//chassis.moveToPose(0, 9, 80, 900,{.minSpeed = 100});
	//chassis.moveToPose(5, 10, 90, 1000,{.minSpeed = 100});
	delay(600);
	arm.move(127);
	delay(600);
	arm.move(0);
	chassis.moveToPose(8, 5, 105, 1200,{.minSpeed = 100});
	// chassis.waitUntilDone();
	chassis.moveToPose(10, 2.5, 70, 1000,{.minSpeed = 100});
	chassis.waitUntilDone();
	chassis.turnToHeading(80, 400);
	arm.move(-127);
	delay(450);
	arm.move(0);
	delay(600);


	*/
	// chassis.moveToPose(22, 5, 90, 1000,{.minSpeed = 100});
	
	// chassis.moveToPose(-27, 5, 90, 1000,{.forwards = false});
	// chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	// firstIntake.move(0);
	// secondIntake.move(0);


	//chassis.moveToPose(-30.8,-12.6,330,1600);
	// chassis.moveToPose(-36,-18,333,1100);
	// chassis.waitUntilDone();
	// chassis.moveToPose(-40, -35, 330, 1000);
	// chassis.waitUntilDone();
	// chassis.setPose(0,0,0);
	// chassis.moveToPose(0,-20,0,900,{.forwards = false, .minSpeed = 50});
	// chassis.turnToHeading(270, 400);
	// firstIntake.move(-127);
	// secondIntake.move(-127);
	// delay(500);
	// chassis.turnToHeading(0, 300);
	// delay(600);
	// arm.move(127);
	// delay(600);
	// arm.move(0);
	// chassis.moveToPose(0,20, 0, 900);
	// firstIntake.move(127);
	// secondIntake.move(0);
	// chassis.moveToPose(0, -10, 0, 900,{.forwards = false,.minSpeed = 50});
	// chassis.turnToHeading(290, 200);
	// firstIntake.move(-127);
	// secondIntake.move(-127);
	// chassis.waitUntilDone();
	// chassis.turnToHeading(0, 200);
	// chassis.moveToPose(0, 20, 0, 900);
	// delay(500);
	// arm.move(-127);
	// delay(300);
	// arm.move(0);


	
	//firstIntake.move(127);
	//secondIntake.move(127);
	// chassis.moveToPose(-38, 3, 155, 1200);
	// chassis.waitUntilDone();
	// chassis.moveToPose(-38, 1, 155, 1400);
	// delay(1500);

	// chassis.moveToPose(-38, -10,155,1200,{.forwards = false, .maxSpeed = 30});
	
	//chassis.moveToPose(-24, -9, 300, 2000,{.maxSpeed = 80});
	//chassis.moveToPose(-26, -14, -75, 1200);
	//chassis.moveToPose(-33,-23,-81,1000);
	//chassis.turnToHeading(270, 1200);


}

// BLUE SIDE

void blueRush(){
	chassis.setPose(0,0,0);
    chassis.moveToPose(0, -27.37, 0, 2000,{.forwards = false, .minSpeed = 105, .earlyExitRange = 4});
    // chassis.moveToPose(10.5, -45.5, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
    chassis.moveToPose(-10.5, -45, 29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.waitUntilDone();
    pros::delay(100);
    clamp.set_value(1);
    //pros::delay(200);
    chassis.turnToHeading(20, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(-10.5, -30.5, -11, 1500,{.minSpeed = 127});
    chassis.moveToPose(-10.3, 0, -11, 2000,{.minSpeed = 127});
    chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
    chassis.waitUntil(15);
    firstIntake.move(127);
    secondIntake.move(127);
    delay(200);
    chassis.moveToPose(-10.3, -25, -11, 2000,{.forwards = false, .minSpeed = 50});
    delay(600);
    firstIntake.move(0);
    secondIntake.move(0); // only for testing purposes
    chassis.waitUntilDone();
    //chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 300);
    //chassis.waitUntilDone();
    clamp.set_value(0);
    chassis.turnToHeading(-275, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(-51, -25, -275, 1000,{.forwards = false, .minSpeed = 40});
    chassis.waitUntilDone();
    clamp.set_value(1);
    arm.move_absolute(3000, 127);
    firstIntake.move(127);
    secondIntake.move(127);
    chassis.turnToHeading(-50, 300);
    chassis.moveToPose(-63, 5, -50, 1800);
    chassis.moveToPose(-51, -15, 0, 1000,{.forwards = false, .minSpeed = 127});
    chassis.waitUntilDone();
    arm.move_absolute(-1, 80);
    delay(1200);
    clamp.set_value(0);
    delay(500);
    chassis.moveToPose(-55,-30,-180,1300);
    firstIntake.move(0);
    //secondIntake.move(0);
}

void blueRush2(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -27.37, 0, 2000,{.forwards = false, .minSpeed = 105, .earlyExitRange = 4});
	// chassis.moveToPose(10.5, -45.5, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.moveToPose(-10.5, -45, -29, 2000,{.forwards = false,.lead = 0.3, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(300);
	clamp.set_value(1);
	pros::delay(200);
	chassis.turnToHeading(20, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(-10.5, -30.5, 11, 1500,{.minSpeed = 127});
	chassis.moveToPose(-10.3, 0, 11, 2000,{.minSpeed = 127});
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntil(15);
	firstIntake.move(127);
	secondIntake.move(127);
	delay(200);
	chassis.moveToPose(-10.3, -25, 11, 2000,{.forwards = false, .minSpeed = 50});
	delay(600);
	firstIntake.move(0);
	secondIntake.move(0); // only for testing purposes
	chassis.waitUntilDone();
	//chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 300);
	//chassis.waitUntilDone();
	clamp.set_value(0);
	chassis.turnToHeading(275, 500);
	chassis.waitUntilDone();
	chassis.moveToPose(-51, -25, 275, 1000,{.forwards = false, .minSpeed = 40});
	chassis.waitUntilDone();
	clamp.set_value(1);
	arm.move_absolute(3000, 127);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.turnToHeading(50, 300);
	chassis.moveToPose(-63, 5, 50, 1800);
	chassis.moveToPose(-51, -15, 0, 1000,{.forwards = false, .minSpeed = 127});
	chassis.waitUntilDone();
	arm.move_absolute(-1, 80);
	delay(1200);
	clamp.set_value(0);
	delay(500);
	chassis.moveToPose(-55,-30,180,1300);
	firstIntake.move(0);
	//secondIntake.move(0);
}

void blueRush1(){
	chassis.setPose(56.988,-60.793,85);
	chassis.follow(blueSwipe_txt,10,2500,false);
	clamp.set_value(1);
	chassis.turnToHeading(80, 2500);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(22.143,-47.474,35,2500);
	chassis.moveToPoint(47.355, 3.71, 4000);
	chassis.waitUntilDone();
	firstIntake.move(0);
	secondIntake.move(0);
	chassis.turnToHeading(165, 1500);
	chassis.follow(blueStraight_txt,10,4000,true);
	chassis.waitUntil(20);
	firstIntake.move(127);
	secondIntake.move(127);
	
	
}

void blueFar(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0,-25, 0,1200,{.forwards = false, .minSpeed = 50});
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	pros::delay(100);
	clamp.set_value(1);
	//pros::delay(300);
	//chassis.moveToPose(5.4, -25.377, 76, 1000);
	//chassis.turnToHeading(55,300);
	chassis.turnToHeading(-72,400);
	firstIntake.move(127);
	secondIntake.move(127);
	//chassis.moveToPose(20, -1.5, 55, 1300);
	chassis.moveToPose(-22, -15, -60, 1200); // get first on field
	chassis.turnToHeading(-140, 400);
	chassis.moveToPose(-14.3,-20, -130,1000);
	//chassis.moveToPose(20, -30, 156,1300);
	chassis.waitUntilDone();
	chassis.moveToPose(-25, -39, -155, 900);
	//chassis.turnToHeading(185, 300);
	chassis.moveToPose(-18, -25, -155, 1000,{.forwards = false});
	chassis.turnToHeading(-110, 400);
	chassis.moveToPose(-33, -31, -110, 1300);
	chassis.moveToPose(-15, -30, -110, 1000,{.forwards = false});
	delay(900);
	//primary
	chassis.turnToHeading(84, 500);
	//arm.move_absolute(3150,127); for ring on red wall goal 
	//chassis.moveToPose(-20, -21, -83, 2600);
	chassis.moveToPose(12,-38,-130,1400,{.maxSpeed = 70});
	chassis.waitUntil(10);
	clamp.set_value(0);
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
}


// ----------------------------- Auto Functions --------------------------------- // 

void skillz(){
	chassis.setPose(-58.367,0,270); // begin point
	//first corner (red neg)
	arm.move(127);
	delay(400);
	arm.move(0);
	chassis.moveToPose(-71.5, 0, 270, 900);
	chassis.waitUntilDone();
	delay(200);
	arm.move(-127);
	delay(500);
	arm.move(0);


	//chassis.moveToPose(-58, 0, 270, 1000,{.forwards = false, .minSpeed = 127});
	//chassis.moveToPose(-47,5,254,3000,{.forwards = false, .minSpeed = 100});
	chassis.moveToPose(-50, 0, 270, 3000,{.forwards = false, .minSpeed = 100});
	// chassis.moveToPose(-49, 5, 155, 3000,{.forwards = false, .minSpeed = 100});
	chassis.turnToHeading(200, 500);
	chassis.waitUntilDone();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.follow(skills1_txt, 8, 2100, false);
	// chassis.moveToPose(-43, 10, 212, 1000,{.forwards = false});
	// chassis.moveToPose(-48, 17, 150, 1000,{.forwards = false});
	// chassis.moveToPose(-50, 18, 165, 1000,{.forwards = false});
	delay(1400);
	clamp.set_value(1);
	delay(400);
	chassis.turnToHeading(90, 600);
	chassis.waitUntilDone();
	//chassis.follow(skills2_txt, 10, 2000,true);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.moveToPose(-38, 23, 90, 2000,{.minSpeed = 120});
	chassis.waitUntilDone();
	//chassis.moveToPose(0, 50, 10,1500);
	// chassis.moveToPose(0.143, 59.081, 45, 1500,{.minSpeed = 100});
	// chassis.waitUntilDone();
	// chassis.turnToHeading(241, 400);
	//chassis.moveToPose(-29, 27, 35, 2000,{.minSpeed = 127});
	
	chassis.turnToHeading(-5, 1000);
	//chassis.moveToPose(-8,80,40,2500);
	//chassis.moveToPose(1.5, 64, 33, 1500);
	chassis.moveToPose(-37, 51, -5, 1800);
	chassis.turnToHeading(50, 400);
	chassis.waitUntilDone();
	chassis.moveToPose(-15.3, 62, 50, 2000,{.minSpeed = 90});
	chassis.moveToPose(-15.3, 48, 50, 200,{.forwards = false});
	chassis.waitUntilDone();
	//chassis.moveToPose(-15, 55, 50, 1400,{.forwards = false});
	//chassis.waitUntilDone();
	//chassis.turnToHeading(90, 500);
	chassis.moveToPose(-69, 69, 106, 2500,{.minSpeed = 30});
	chassis.waitUntilDone();
	chassis.moveToPoint(-37, 60, 2000,{.forwards = false});
	chassis.waitUntilDone();
	chassis.moveToPose(-56.832,49.096,190,1900);
	chassis.waitUntilDone();
	chassis.moveToPose(-80,50,280,1800);
	chassis.waitUntilDone();
	delay(600);
	chassis.moveToPose(-90.184,73.589,-60,1600,{.forwards = false,.minSpeed = 60});


	// // chassis.waitUntilDone();
	// // chassis.moveToPose(-43, 55, 180, 1500);
	// // chassis.waitUntilDone();
	// // chassis.moveToPose(-42, 55,270,1300);
	//firstIntake.move(-100);
	delay(800);
	secondIntake.move(-100);
	delay(200);
	firstIntake.move(0);
	secondIntake.move(0);
	clamp.set_value(0);
	//delay(700);
	chassis.moveToPoint(-70,45,1300);
	//chassis.moveToPose(-55, 52, 320, 1000,{.forwards = false});
	//chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);//stop working 
	// chassis.follow(skills3_txt, 10, 3600,false);
	// chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	// only if the pure pursuit doesnt work 
	//chassis.moveToPose(-39,47,0,1600,{.forwards = false});
	//chassis.moveToPoint(-40, 47, 1600, {.forwards = false});
//	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	//chassis.turnToHeading(50,600);
	chassis.waitUntilDone();
	chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	chassis.waitUntilDone();
	chassis.moveToPose(-61.69, -15.9, 33.8, 2600,{.forwards = false, .lead = 0.3, .maxSpeed = 85});
//	chassis.moveToPose(-60, -15.248, -335, 2600,{.forwards = false, .lead = 0.3, .maxSpeed = 85});
	chassis.waitUntilDone();
	delay(1200);
	clamp.set_value(1);
	delay(400);
	firstIntake.move(127);
	secondIntake.move(127);
	delay(500);
	chassis.moveToPose(-22.42, -19.522, 90, 1400); // x = 30 before y = 24 before
	chassis.moveToPose(-41, -38, 190, 2000);
	chassis.moveToPose(-60, -38, 300, 1300);
	chassis.moveToPose(-75, -38, 300, 1400);
	chassis.moveToPose(-50, -38, 300, 1400,{.forwards = false});
	chassis.moveToPose(-50, -63, 100, 1400);
	chassis.moveToPose(-80, -80, 50, 1400,{.forwards = false, .lead = 0.82});
	chassis.waitUntilDone();
	delay(800);
	clamp.set_value(0);
	delay(600);
	chassis.moveToPose(40, 5, -305, 4000); //orgianl x = 40
	chassis.waitUntilDone();
	firstIntake.move(0);
	secondIntake.move(0);
	chassis.moveToPose(30, 90, -350, 3000); //original x = 45
	chassis.moveToPose(30, -90, -350, 3000,{.forwards = false});//original x = 45
	//chassis.moveToPose(35, 90, -358, 3000);//original x = 45
	//chassis.moveToPose(-1, 50, 250, 2000);

	/*
	chassis.moveToPose(25, -38, 50, 1300);
	firstIntake.move(127);
	secondIntake.move(127);
	while(distance.get_distance() <= 55){
		firstIntake.move(0);
		secondIntake.move(0);
	}
	firstIntake.move(-90);
	secondIntake.move(-90);
	delay(1200);
	firstIntake.move(0);
	secondIntake.move(0);
	chassis.moveToPose(-6,-50,200,1400);
	arm.move_absolute(3000, 127);
	chassis.waitUntilDone();
	arm.move(-127);
	delay(550);
	arm.move(0);
	*/




	//chassis.moveToPose(-60, -, float theta, int timeout)
	
}



void autonomous() {
	
//tuning angular : 
	// chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// chassis.setPose(0, 0, 0);
	// chassis.turnToHeading(90, 1000);
	// chassis.waitUntilDone();
//tuning lateral : 
	// chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// chassis.setPose(0, 0, 0);
	// chassis.moveToPoint(0, -35, 1000,{.forwards = false});
	// chassis.moveToPoint(0, 0, 1000);
	// chassis.waitUntilDone();
//arm : 
	// arm.move(127);
	// delay(900);
	// arm.move(0);
	// chassis.setPose(0,0,0);
	// chassis.moveToPose(0,4,0,1000);
	// arm.move_absolute(3000, 127);
	// firstIntake.move(127);
	// secondIntake.move(127);
	// chassis.moveToPose(0, -5, 0, 1500,{.forwards = false});
	// delay(8000);
	// arm.move_absolute(-100, 127);

//	---- RED SIDE ----	// 
	//redRush();
	//redFar();

//	---- BLUE SIDE ----	//
	//blueRush();
	//blueFar();
// ---- SKILLZ ----- //
	skillz();





	// chassis.setPose(0,0,0);
	// chassis.moveToPose(0,-25, 0,1200,{.forwards = false, .minSpeed = 50});
	// chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// chassis.waitUntilDone();
	// pros::delay(300);
	// clamp.set_value(1);
	// pros::delay(300);
	// chassis.turnToHeading(72,400);
	// chassis.moveToPose(13, -18, 73, 1000);



	//chassis.moveToPose(5.4, -25.377, 76, 1000);
	// chassis.turnToHeading(55,300);
	// firstIntake.move(127);
	// secondIntake.move(127);
	// chassis.moveToPose(22, -15, 45, 1200); // get first on field
	// chassis.turnToHeading(150, 400);
	// chassis.moveToPose(14.3,-20, 130,950);
	// //chassis.moveToPose(20, -30, 156,1300);
	// chassis.waitUntilDone();
	// chassis.moveToPose(25, -39, 155, 950);
	// //chassis.turnToHeading(185, 300);
	// chassis.moveToPose(18, -25, 155, 700,{.forwards = false});
	// chassis.turnToHeading(110, 400);
	// chassis.moveToPose(33, -30, 110, 1000);
	// chassis.moveToPose(15, -30, 110, 800,{.forwards = false});
	// delay(900);
	// //primary
	// chassis.turnToHeading(-84, 500);
	// arm.move_absolute(3150,127);
	// //chassis.moveToPose(-20, -21, -83, 2600);
	// chassis.moveToPose(-25,-17,240,3000,{.maxSpeed = 100});
	// chassis.waitUntil(20);
	// clamp.set_value(0);
	// chassis.moveToPose(-28.5, -20.5, 240,1000,{.forwards = false});
	// chassis.waitUntilDone();
	// arm.move_absolute(-20, 127);
	// delay(500);
	// while(distance.get() >= 35){
	// 	secondIntake.move(90);
	// }
	// firstIntake.move(-90);
	// secondIntake.move(-90);
	// delay(500);
	// chassis.setPose(0,0,0);
	// chassis.moveToPose(0, -10, 0,1200,{.forwards = false, .minSpeed = 60});
	// firstIntake.move(-127);
	// secondIntake.move(0);
	// chassis.waitUntilDone();
	// chassis.moveToPose(8, 13, 10, 1200,{.minSpeed = 100});
	// chassis.waitUntilDone();
	// chassis.moveToPose(5, 10, 90, 1000,{.minSpeed = 100});
	// delay(600);
	// arm.move(127);
	// delay(600);
	// arm.move(0);
	// chassis.moveToPose(13, 5, 90, 1000,{.minSpeed = 100});
	// chassis.waitUntilDone();
	// chassis.moveToPose(22, 5, 90, 1000,{.minSpeed = 100});
	// arm.move(-127);
	// delay(450);
	// arm.move(0);
	// delay(600);
	// chassis.moveToPose(-27, 5, 90, 1000,{.forwards = false});
	// chassis.setBrakeMode(E_MOTOR_BRAKE_BRAKE);
	// firstIntake.move(0);
	// secondIntake.move(0);




}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 
void opcontrol() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	bool clampState = false; 
	bool hangState = false;
	bool state = false;
	bool mode1 = false;
	bool isArcade = false;
	int time = 20;
	while(true){
		//double stick 
		int DoubleLeftY = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int DoubleRightX = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		//tank 
		int LTankY = master.get_analog(ANALOG_LEFT_Y);
        int RTankY = master.get_analog(ANALOG_RIGHT_Y);
		//leftMotor.move(leftY + rightX);                      // Sets left motor voltage
		//rightMotor.move(leftY - rightX);                     // Sets right motor voltage
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
			chassis.tank(LTankY, RTankY);
		}else{
			leftMotor.move(DoubleLeftY + DoubleRightX);
			rightMotor.move(DoubleLeftY - DoubleRightX);
		}


		if(master.get_digital(DIGITAL_R1)){
			arm.move(127);
		}
		else if(master.get_digital(DIGITAL_R2)){
			arm.move(-127);
		}
		else{
			arm.move(0);
		}

		// if(master.get_digital(DIGITAL_DOWN)){
		// 	state = true;
		// }
		// if(master.get_digital(DIGITAL_UP)){
		// 	state = false;
		// }

		//int intakeSpeed = state ? 100 : 127;
        if(master.get_digital(DIGITAL_L1)){
            firstIntake.move(127);
            secondIntake.move(127);
        }else if(master.get_digital(DIGITAL_L2)){
            firstIntake.move(-127);
            secondIntake.move(-127);
        }else if(master.get_digital_new_press(DIGITAL_UP)){
			while(distance.get_distance() >=  55){
				firstIntake.move(100);
				secondIntake.move(100);
			}
			firstIntake.move(100);
			secondIntake.move(100);
			while(time >= 0){
				secondIntake.move(-127);
				time -= 1;
			}
			
		}else if(master.get_digital(DIGITAL_X)){
			firstIntake.move(100);
			secondIntake.move(100);
			if(distance.get_distance() <= 55){ // using distance sensor
				firstIntake.move(0);
				secondIntake.move(0);
			}
			// while(distance.get_distance() >=55){
			// 	leftMotor.move(DoubleLeftY + DoubleRightX);
			// 	rightMotor.move(DoubleLeftY - DoubleRightX);

			// 	firstIntake.move(100);
			// 	secondIntake.move(100);
				

			// 	if(master.get_digital_new_press(DIGITAL_B)){
			// 		clampState = !clampState;
			// 		clamp.set_value(clampState);
			// 	}
			// 	if(master.get_digital_new_press(DIGITAL_A)){
			// 		hangState = !hangState;
			// 		hang.set_value(hangState);
			// 	}


			// 	if(master.get_digital(DIGITAL_R1)){
			// 		arm.move(127);
			// 	}
			// 	else if(master.get_digital(DIGITAL_R2)){
			// 		arm.move(-127);
			// 	}
			// 	else{
			// 		arm.move(0);
			// 	}
			// }
		}else{
            firstIntake.move(0);
            secondIntake.move(0);
        }

		/*
		alternate for the "int intakeSpeed = STate ? 80 : 127"
		if(state == false){
			if(master.get_digital(DIGITAL_L1)){
			firstIntake.move(127);
			secondIntake.move(127);
			}else if(master.get_digital(DIGITAL_L2)){
				firstIntake.move(-127);
				secondIntake.move(-127);
			}else{
				firstIntake.move(0);
				secondIntake.move(0);
			}
		}else{
			if(master.get_digital(DIGITAL_L1)){
				firstIntake.move(80);
				secondIntake.move(80);
			}else if(master.get_digital(DIGITAL_L2)){
				firstIntake.move(-80);
				secondIntake.move(-80);
			}else{
				firstIntake.move(0);
				secondIntake.move(0);
			}
		}
		*/
		if(master.get_digital(DIGITAL_LEFT)){
			firstIntake.move(127);
		}
		if(master.get_digital(DIGITAL_Y)){
			secondIntake.move(127);
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			clampState = !clampState;
			clamp.set_value(clampState);
		}
		if(master.get_digital_new_press(DIGITAL_A)){
			hangState = !hangState;
			hang.set_value(hangState);
		}
		pros::delay(20);
	}
}