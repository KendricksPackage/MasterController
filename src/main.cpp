#include "main.h"
#include "lemlib/api.hpp"
//#include "pros/misc.h"
//#include <utility>
using namespace pros;
Controller master(E_CONTROLLER_MASTER);

pros::Motor firstIntake(13);
pros::Motor secondIntake(1);
pros::Motor LeftArmMotor(9);
pros::Motor RightArmMotor(-11);
pros::adi::DigitalOut clamp(1);

//Motor group
pros::MotorGroup leftMotor({-7,-2},pros::MotorGears::blue);
pros::MotorGroup rightMotor({15,6},pros::MotorGears::blue);
pros::MotorGroup arm({9,-11});

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
							10, // proportional gain (kP)
							0, // integral gain (kI)
							3, // derivative gain (kD)
							3, // anti windup
							1, // small error range, in inches
							100, // small error range timeout, in milliseconds
							3, // large error range, in inches
							500, // large error range timeout, in milliseconds
							20 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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
lemlib::TrackingWheel trackwheel(&vert, lemlib::Omniwheel::NEW_275,0);

pros::v5::IMU imu(10);

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

	pros::lcd::register_btn1_cb(on_center_button);

	while (true) { // infinite loop
        // print measurements from the adi encoder
  		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::delay(20); // delay to save resources. DO NOT REMOVE
    }
	
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
ASSET(swipe_txt);
ASSET(straight_txt);



// ----------------------------- Auto Functions --------------------------------- // 
void redRush(){
	//chassis.setPose(-147.648,-56.036,280);
	//chassis.follow(swipe_txt,10,2500,true);
	/*clamp.set_value(false);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.turnToHeading(275,1000);
	chassis.moveToPose(-23.166,-47.474,330,2000,{true});
	firstIntake.move(0);
	secondIntake.move(0);
	chassis.moveToPose(-46.951,0.095,350);
	firstIntake.move(127);
	secondIntake.move(127);
	chassis.turnToHeading(200,1000);
	chassis.follow(straight_txt,2500,true);
	chassis.TurnToHeading()
*/

}





void autonomous() {
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0, 10, 3000);
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
	bool toggle = true; 
	bool state = false;
	bool mode1 = false;
	bool mode2 = false;

	while(true){
		int leftY = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int rightX = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		leftMotor.move(leftY + rightX);                      // Sets left motor voltage
		rightMotor.move(leftY - rightX);                     // Sets right motor voltage
		if(master.get_digital(DIGITAL_R1)){
			arm.move(127);
		}
		else if(master.get_digital(DIGITAL_R2)){
			arm.move(-127);
		}
		else{
			arm.move(0);
		}
		if(master.get_digital(DIGITAL_DOWN)){
			state = true;
		}if(master.get_digital(DIGITAL_UP)){
			state = false;
		}

		int intakeSpeed = state ? 80 : 127;
        if(master.get_digital(DIGITAL_L1)){
            firstIntake.move(intakeSpeed);
            secondIntake.move(intakeSpeed);
        } 
        else if(master.get_digital(DIGITAL_L2)){
            firstIntake.move(-intakeSpeed);
            secondIntake.move(-intakeSpeed);
        } 
        else{
            firstIntake.move(0);
            secondIntake.move(0);
        }

		/*
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
		if(master.get_digital(DIGITAL_A)){
			mode1 = true;
		}if(master.get_digital(DIGITAL_Y)){
			mode1 = false;
			mode2 = true;
		}
		if(mode1){
			if(master.get_digital(DIGITAL_L1)){
				firstIntake.move(127);
			}else if(master.get_digital(DIGITAL_L2)){
				firstIntake.move(-127);
			}else{
				firstIntake.move(0);
			}
		}else if(mode2){
			if(master.get_digital(DIGITAL_L1)){
				secondIntake.move(127);
			}else if(master.get_digital(DIGITAL_L2)){
				secondIntake.move(-127);
			}else{
				secondIntake.move(0);
			}
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			clamp.set_value(toggle);
			toggle = !toggle;
		}
		pros::delay(20);
	}
}