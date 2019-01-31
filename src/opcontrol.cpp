#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
using namespace okapi::literals;
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




 //auto drive = ChassisControllerFactory::create()

auto drive = okapi::ChassisControllerFactory::create(
		 {1, 2}, {-3, -9},
		 okapi::AbstractMotor::gearset::green,
		 {4_in, 12_in}
 );
 auto profileController = AsyncControllerFactory::motionProfile(
   1.0,  // Maximum linear velocity of the Chassis in m/s
   2.0,  // Maximum linear acceleration of the Chassis in m/s/s
   9.0, // Maximum linear jerk of the Chassis in m/s/s/s
   drive // Chassis Controller
 );



int flywheelToggle = 0;
okapi::Motor lift_mtr(-5);
okapi::Motor roller_mtr(7);
okapi::MotorGroup flywheel({-6, -8});
//roller_mtr.AsyncVelPIDController()
okapi::Motor TopFlywheel(-6);
okapi::Motor BottomFlywheel(-8);
okapi::Controller controller;
okapi::ADIEncoder Flywheel('A', 'B', true);
void flywheelTask(void *param);
int lcdCounter = 4 ;
//------------------------------------------------------------------------------------------//
struct PID
{
    float kP;
    float kI;
    float kD;
    float error;
    float integral;
    float derivative;
    float previous_error;
    float speed;
    float target;
    float sensor;
		float fw;
};
struct PID FW;


void flywheelPid(void *)
{
    while (true)
    {
        FW.kP = 0.4;
        FW.kD = 1.55;
        FW.kI = 0.0001;
        FW.sensor = Flywheel.get() * 25;
				FW.fw = 0;
        std::cout << "RPM: " << FW.sensor << std::endl;
				std::cout << "temp:" << BottomFlywheel.getTemperature();
        Flywheel.reset();
        FW.error = FW.target - FW.sensor;
        FW.derivative = FW.error - FW.previous_error;
        FW.integral += FW.error;
        FW.previous_error = FW.error;
        FW.speed = FW.kP * FW.error + FW.kD * FW.derivative + FW.kI * FW.integral + FW.fw;
				if(FW.sensor > FW.target + 100){
					FW.speed -= 5;
				}
        if (controller.getDigital(ControllerDigital::A))
        {
            FW.speed = -2;
        }
        else if (flywheelToggle == 0)
        {
            FW.speed = 0;
        }
        else if (FW.speed < 0.5)
        {
            FW.speed = 0.5;
        }
				if(FW.target > 3000){
					TopFlywheel.moveVelocity(200);
					BottomFlywheel.moveVelocity(200);
				}
				else{
					TopFlywheel.controllerSet(FW.speed);
					BottomFlywheel.controllerSet(FW.speed);
				}


		//		TopFlywheel.moveVoltage(FW.target*3.75);
		//		TopFlywheel.moveVoltage(FW.target*3.75);


        pros::delay(20);

    }
}


		//controller.setText(0,0, "speed:" +  std::to_string(Flywheel.get()));
//*------------------------------------------------------------------------------------------------------*//

														//DRIVE

void opcontrol() {

pros::Task flywheelTaskHandle(flywheelTask);
pros::Task flywheelpid(flywheelPid);
//flywheel.setVelPID(0,0,0,99);

int flywheelToggle = 0;
std::string speed = "";
while (true) {
		//flywheelPid();

		drive.arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
		roller_mtr.moveVelocity( 600 * controller.getDigital(ControllerDigital::R2) - 600 * controller.getDigital(ControllerDigital::R1));
		lift_mtr.moveVelocity( 200 * controller.getDigital(ControllerDigital::L1) - 200 * controller.getDigital(ControllerDigital::L2));
		controller.setText(0,0, "speed:" +  std::to_string(TopFlywheel.getActualVelocity()));
		//controller.setText(0,0, "speed:" +  std::to_string(vel));
		pros::delay(20);




	}
}
//*------------------------------------------------------------------------------------------------------*//
//flywheel toggle
void flywheelTask(void *)
{
	while (true)
	{
			if (controller.getDigital(ControllerDigital::up))
			{
					pros::delay(20);
					flywheelToggle++;
					if (flywheelToggle > 2)
					{
						flywheelToggle = 0;
					}
					switch (flywheelToggle)
					{
						case 0:
						FW.target = 0;
						break;
						case 1:
						FW.target = 2800;

						break;
					case 2:
				FW.target = 4200;


				break;
				}

				while (controller.getDigital(ControllerDigital::up))
				{
				pros::delay(30);
				}
				pros::delay(50);
				}
				}
	}//end flywheelytask
//*------------------------------------------------------------------------------------------------------*//
															//AUTONS initialize
void redBack();
void blueBack();
void redFront();
void blueFront();
void progSkills();

void autonomous()
{

	switch(lcdCounter)
		{
		case 1:
			blueFront();
			break;
		case 2:
			redFront();
			break;
		case 3:
			blueBack();
			break;
		case 4:
			redBack();
			break;
		case 5:
			progSkills();
			break;
		}

}
//*------------------------------------------------------------------------------------------------------*//
//                                             auton functions
void move(Point target){
	profileController.generatePath({Point{0_ft, 0_ft, 0_deg}, target},  "m");
	profileController.setTarget("m");
	profileController.waitUntilSettled();

}
void moveb(Point target){
	profileController.generatePath({Point{0_ft, 0_ft, 0_deg}, target}, "a");
	profileController.setTarget("a", true);//true inverts drive
	profileController.waitUntilSettled();
}
void roller(float rotations){
	roller_mtr.move_relative(rotations*300, 600);//conversion for 900 ticks per rotation
	while(roller_mtr.getPosition() < rotations*300){
		pros::delay(20);

	}
}
//*------------------------------------------------------------------------------------------------------*//
//                                    Autons

void redBack()
	{
		flywheelToggle++;
		pros::Task flywheelTaskHandle(flywheelTask);
		pros::Task flywheelpid(flywheelPid);


		drive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	/*	move(Point{3.2_ft,0_ft, 0_deg});//move toward cap

		roller(1.4);//intake ball
		lift_mtr.moveRelative(-3*900, 200);//put out cap flipper
		drive.moveDistance(-5_in);//back up a bit
		drive.turnAngle(-120_deg);//turn to face cap
		drive.moveDistance(-0.5_ft);//move back into cap a bit
		lift_mtr.moveRelative(900*0.6, 200);//flip cap
		drive.turnAngle(0);//face flags
		roller(1.5);//shoot
		drive.turnAngle(-10_deg);//turn to face platofrm
		drive.moveDistance(3_ft);//park

*/
	}
void blueBack(){}
void redFront(){

}
void blueFront(){}
void progSkills(){}
/*----------------------------------------------------------------------------------------*/
bool selected = true;

void left_button()
{
	if (!selected)
	{
		lcdCounter--;
		if (lcdCounter < 0)
		{
			lcdCounter = 0;
		}
	}
}
void center_button()
{
	if (!selected)
	{
		selected = true;
	}
}
void right_button()
{
	if (!selected)
	{
		lcdCounter++;
		if (lcdCounter > 5)
		{
			lcdCounter = 5;
		}
	}
}
std::string convert(int arg)
{
	switch (arg)
	{
	case 1:
		return "Blue Front";
	case 2:
		return "Red Front";
	case 3:
		return "Blue Back";
	case 4:
		return "Red Back";
	case 5:
		return "Prog Skills";
	default:
		return "No Auton";
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(left_button);
	pros::lcd::register_btn1_cb(center_button);
	pros::lcd::register_btn2_cb(right_button);
	while (!selected)
	{
			pros::lcd::set_text(0, convert(lcdCounter));
				pros::delay(20);
	}

	pros::lcd::set_text(0, convert(lcdCounter) + " (SELECTED)");
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
