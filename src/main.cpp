#include "main.h"
#include "EZ-Template/auton.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_core/lv_style.h"
#include "display/lv_fonts/lv_font_builtin.h"
#include "display/lv_misc/lv_color.h"
#include "display/lv_misc/lv_symbol_def.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_cont.h"
#include "display/lv_objx/lv_gauge.h"
#include "display/lv_objx/lv_label.h"
#include "display/lv_objx/lv_list.h"
#include "display/lv_objx/lv_page.h"
#include "display/lv_objx/lv_tabview.h"
#include "gif-pros/gifclass.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <string>
#include <map>
#include <utility>



pros::Motor cata (11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake (12, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveFR (13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveMR (15, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveBR (14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveFL (16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveML (18, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveBL (17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);


pros::ADIDigitalIn cata_limit ('A');

pros::Rotation rotation_left(20);
pros::Rotation rotation_right(19);

pros::Optical optical(16);

pros::Imu inertial(10);

// pros::Vision vision(13);

pros::Controller master (pros::E_CONTROLLER_MASTER);


std::map<std::string, pros::Motor> motorMap;
pros::Motor motorArr[] = {cata, intake, driveFR, driveMR, driveBR, driveFL, driveML, driveBL};

/**
 * Called when a button is released
 * @param btn pointer to the released button
 * @return LV_RES_OK because the object is not deleted in this function
 */
static  lv_res_t btn_rel_action(lv_obj_t * btn)
{
    /*Increase the button width*/
    lv_coord_t width = lv_obj_get_width(btn);
    lv_obj_set_width(btn, width + 20);

    return LV_RES_OK;
}




static void disconnect_detection(void* param) {

	std::map<std::string, pros::Motor>::iterator it;

	while (true) {
			for (it = motorMap.begin(); it != motorMap.end(); it++) {
			if (it->second.get_current_draw() > 100000000) {
				pros::delay(100);

				master.print(1,0, "%s", it->first);
				pros::delay(100);

				master.rumble(". . ");
				pros::delay(100);

			} else {
				pros::delay(100);

				master.rumble(" ");
				pros::delay(100);

				master.clear_line(1);
				pros::delay(100);

			}
		}
		pros::delay(500);
	}
}

bool manualCataFlag = true;
// 1 is pressed, 0 is open
 void cata_task(void* param) {
			cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && manualCataFlag) {
			cata = -127;
      intake = 127;			
		}  else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			manualCataFlag = false;
				
			// launch, limit will be pressed at start
			while (cata_limit.get_value() == 1) {
							cata = -127;
      intake = 127;		
        
				pros::delay(20);
			}

			pros::delay(200);
			
			
			// wind back and reset, limit will be open at start
			while (cata_limit.get_value() == 0) {
							cata = -127;
           intake = 127;		
				pros::delay(10);
			}

			cata = 0;
      intake = 0;

			manualCataFlag = true;

		} else {
			cata = 0;
      intake = 0;
		} 
		pros::delay(20);
	}
 }


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-16, 18, -17}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{13, -15, 14}

  // IMU Port
  ,10

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6. 60:
  // 48 powered 72 driven
  ,0.6666666

  // Uncomment if using tracking wheels
  
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,20 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-19 // Rotation sensor
  

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

void Drive::initialize(bool imu) {
  init_curve_sd();
  reset_drive_sensor();
}


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;


void skills() {
  // always one inch off
  // chassis.set_drive_pid(23, DRIVE_SPEED, true, true);
  // chassis.wait_drive();

  // chassis.set_turn_pid(90, TURN_SPEED);
  // chassis.wait_drive();

  // chassis.set_turn_pid(-90, TURN_SPEED);
  // chassis.wait_drive();

  //   chassis.set_turn_pid(0, TURN_SPEED);
  // chassis.wait_drive();



  // first 3 match loads

    chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();

  pros::delay(1000); // shoot here

      chassis.set_swing_pid(ez::LEFT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_tank(-50 , -50);
  pros::delay(1000);
  chassis.set_tank(0 , 0);
 

      chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();

  pros::delay(1000); // shoot here

      chassis.set_swing_pid(ez::LEFT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
  
   chassis.set_tank(-50 , -50);
  pros::delay(1000);
  chassis.set_tank(0 , 0);


      chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();

  pros::delay(1000);

      chassis.set_swing_pid(ez::LEFT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_tank(-50 , -50);
  pros::delay(1000);
  chassis.set_tank(0 , 0);

}



void right_elims() {

}

void left_elims() {

}

void solo_WP() {

}


// REMINDER - ddlist things
static lv_obj_t *tabview;

static lv_res_t ddlist_action(lv_obj_t * ddlist)
{
uint8_t id = lv_obj_get_free_num(ddlist);
char sel_str[32];
lv_ddlist_get_selected_str(ddlist, sel_str);
printf("Ddlist %d new option: %s \n", id, sel_str);

   uint32_t sel = lv_ddlist_get_selected(ddlist);
   
    lv_tabview_set_tab_act(tabview, sel, true);

return LV_RES_OK; /*Return OK if the drop down list is not deleted*/
}


static lv_obj_t *ta1;

// REMINDER - BUTTON CLICK ACTION
int autoCounter = 1;
std::string autoName = "x Prog Skills";
std::string autoDesc = "- x volleys\n - x rollers \n - x tile expansion";

static lv_res_t btn_click_action(lv_obj_t * btn)
{
uint8_t id = lv_obj_get_free_num(btn);
printf("Button %d is released\n", id);
/* The button is released.
* Make something here */

switch (id) {
  case 1:
    autoCounter = 1;
    autoName = "x Prog Skills";
    autoDesc = "- x volleys\n - x rollers \n - x tile expansion";
    break;
  case 2:
    autoCounter = 2;
    autoName = "Right Side Elims";
    autoDesc = "right roller \n 3 volleys";
    break;
  case 3:
    autoCounter = 3;
    autoName = "Left Side Elims";
    autoDesc = "- left roller \n - x volleys";
    break;
  case 4:
    autoCounter = 4;
    autoName = "Solo Win Point";
    autoDesc = "- left & right rollers \n - x volleys";
    break;
  default:
    break;
}
    lv_ta_set_text(ta1, (autoName + "\n" + autoDesc).c_str()); /*Set an initial text*/

return LV_RES_OK; /*Return OK if the button is not deleted*/
}




// motor dashboard list click action,  doesn't  really do anything, doesn't need to
static lv_res_t list_release_action(lv_obj_t * list_btn)
{
printf("List element click:%s\n", lv_list_get_btn_text(list_btn));
return LV_RES_OK; /*Return OK because the list is not deleted*/
}



int opticalCounter = 0;
static lv_res_t btn_click_optical(lv_obj_t * btn) {

uint8_t id = lv_obj_get_free_num(btn);
  if (id == 111) {
    if (opticalCounter == 100) {
      opticalCounter = 0;
    } else {
    opticalCounter += 10;

    }
  } else {
    if (opticalCounter == 0) {
      opticalCounter = 0;
    } else {
    opticalCounter -= 10;

    }
  }
  optical.set_led_pwm(opticalCounter);
return LV_RES_OK; /*Return OK if the button is not deleted*/

}


static lv_obj_t *page2;
lv_obj_t *list1; 

int listSpacerCounter = 5;

int tempToggle = 0;
int powerToggle = 0;
int voltToggle = 0;

// REMINDER - DASHBOARD TASK

static lv_obj_t * title;
static lv_obj_t * title2;
static lv_obj_t * title3;
static lv_obj_t * title4;
static lv_obj_t * title5;
static lv_obj_t * title6;
static lv_obj_t * title7;
static lv_obj_t * title8;


static lv_obj_t * title9;
static lv_obj_t * title10;
static lv_obj_t * title11;
static lv_obj_t * title12;
static lv_obj_t * title13;
static lv_obj_t * title14;
static lv_obj_t * title15;
static lv_obj_t * title16;



static lv_obj_t * title17;
static lv_obj_t * title18;
static lv_obj_t * title19;
static lv_obj_t * title20;
static lv_obj_t * title21;
static lv_obj_t * title22;
static lv_obj_t * title23;
static lv_obj_t * title24;





static lv_obj_t *button1;
static lv_obj_t *button2;
static lv_obj_t *button3;
static lv_obj_t *button4;
static lv_obj_t *button5;
static lv_obj_t *button6;

static lv_obj_t *limitBoxLabel2;

static lv_obj_t *rotationBoxLabel;
static lv_obj_t *rotationBoxLabel2;

static lv_obj_t *opticalBoxLabel2;
static lv_obj_t *opticalBoxLabel3;
static lv_obj_t *opticalBoxLabel4;
static lv_obj_t *opticalBoxLabel5;

static lv_obj_t *inertialBoxLabel2;
static lv_obj_t *inertialBoxLabel3;

static lv_obj_t *visionBoxLabel2;
static lv_obj_t *visionBoxLabel3;
static lv_obj_t *visionBoxLabel4;
static lv_obj_t *visionBoxLabel5;


static lv_obj_t * gauge1;


void dashboardTask(void *param) {
  
  inertial.reset();

  pros::delay(2000);

      static lv_style_t rel_style2;
      lv_style_copy(&rel_style2, &lv_style_btn_rel);
      rel_style2.body.radius = 0;
      rel_style2.body.main_color = LV_COLOR_RED;
      rel_style2.body.grad_color = LV_COLOR_RED;


            static lv_style_t rel_style3;
      lv_style_copy(&rel_style3, &lv_style_btn_rel);
      rel_style3.body.radius = 0;
      rel_style3.body.main_color = LV_COLOR_GRAY;

        static lv_style_t rel_style4;
      lv_style_copy(&rel_style4, &lv_style_btn_rel);
      rel_style4.body.radius = 0;
      rel_style4.body.main_color = LV_COLOR_BLUE;
      rel_style4.body.grad_color = LV_COLOR_BLUE;

      static lv_style_t rel_style5;
      lv_style_copy(&rel_style5, &lv_style_btn_rel);
      rel_style5.body.radius = 0;
      rel_style5.body.main_color = LV_COLOR_YELLOW;
      rel_style5.body.grad_color = LV_COLOR_YELLOW;

  while (true) {

  //  temperature = std::to_string(int(motorArr[0].get_temperature())).substr(0,3).c_str(); // horrible line of code: get_temp goes from double to int to string to substring to c_str


  if (tempToggle == 0) {


  lv_btn_set_style(button1, LV_BTN_STYLE_REL, &rel_style2);
  lv_btn_set_style(button2, LV_BTN_STYLE_REL, &rel_style3); // style 3 is gray


    lv_label_set_text(title17, std::to_string(int(motorArr[0].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title18, std::to_string(int(motorArr[1].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title19, std::to_string(int(motorArr[2].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title20, std::to_string(int(motorArr[3].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title21, std::to_string(int(motorArr[4].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title22, std::to_string(int(motorArr[5].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title23, std::to_string(int(motorArr[6].get_temperature())).substr(0,4).c_str());
    lv_label_set_text(title24, std::to_string(int(motorArr[7].get_temperature())).substr(0,4).c_str());
  } else if (tempToggle == 1) {

  lv_btn_set_style(button1, LV_BTN_STYLE_REL, &rel_style3);
  lv_btn_set_style(button2, LV_BTN_STYLE_REL, &rel_style2);


    lv_label_set_text(title17, std::to_string(int(motorArr[0].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title18, std::to_string(int(motorArr[1].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title19, std::to_string(int(motorArr[2].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title20, std::to_string(int(motorArr[3].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title21, std::to_string(int(motorArr[4].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title22, std::to_string(int(motorArr[5].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title23, std::to_string(int(motorArr[6].get_current_draw())).substr(0,4).c_str());
    lv_label_set_text(title24, std::to_string(int(motorArr[7].get_current_draw())).substr(0,4).c_str());
  }



  
  if (powerToggle == 0) {


  lv_btn_set_style(button3, LV_BTN_STYLE_REL, &rel_style4);
  lv_btn_set_style(button4, LV_BTN_STYLE_REL, &rel_style3); // style 3 is gray


    lv_label_set_text(title9, std::to_string(int(motorArr[0].get_power())).substr(0,4).c_str());
    lv_label_set_text(title10, std::to_string(int(motorArr[1].get_power())).substr(0,4).c_str());
    lv_label_set_text(title11, std::to_string(int(motorArr[2].get_power())).substr(0,4).c_str());
    lv_label_set_text(title12, std::to_string(int(motorArr[3].get_power())).substr(0,4).c_str());
    lv_label_set_text(title13, std::to_string(int(motorArr[4].get_power())).substr(0,4).c_str());
    lv_label_set_text(title14, std::to_string(int(motorArr[5].get_power())).substr(0,4).c_str());
    lv_label_set_text(title15, std::to_string(int(motorArr[6].get_power())).substr(0,4).c_str());
    lv_label_set_text(title16, std::to_string(int(motorArr[7].get_power())).substr(0,4).c_str());
  } else if (powerToggle == 1) {

  lv_btn_set_style(button3, LV_BTN_STYLE_REL, &rel_style3);
  lv_btn_set_style(button4, LV_BTN_STYLE_REL, &rel_style4);



    lv_label_set_text(title9, std::to_string(int(motorArr[0].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title10, std::to_string(int(motorArr[1].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title11, std::to_string(int(motorArr[2].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title12, std::to_string(int(motorArr[3].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title13, std::to_string(int(motorArr[4].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title14, std::to_string(int(motorArr[5].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title15, std::to_string(int(motorArr[6].get_actual_velocity())).substr(0,4).c_str());
    lv_label_set_text(title16, std::to_string(int(motorArr[7].get_actual_velocity())).substr(0,4).c_str());
  }





    if (voltToggle == 0) {


  lv_btn_set_style(button5, LV_BTN_STYLE_REL, &rel_style5);
  lv_btn_set_style(button6, LV_BTN_STYLE_REL, &rel_style3); // style 3 is gray


    lv_label_set_text(title, std::to_string(int(motorArr[0].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title2, std::to_string(int(motorArr[1].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title3, std::to_string(int(motorArr[2].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title4, std::to_string(int(motorArr[3].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title5, std::to_string(int(motorArr[4].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title6, std::to_string(int(motorArr[5].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title7, std::to_string(int(motorArr[6].get_voltage())/100).substr(0,4).c_str());
    lv_label_set_text(title8, std::to_string(int(motorArr[7].get_voltage())/100).substr(0,4).c_str());
  } else if (voltToggle == 1) {

  lv_btn_set_style(button5, LV_BTN_STYLE_REL, &rel_style3);
  lv_btn_set_style(button6, LV_BTN_STYLE_REL, &rel_style5);


    lv_label_set_text(title, std::to_string(int(motorArr[0].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title2, std::to_string(int(motorArr[1].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title3, std::to_string(int(motorArr[2].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title4, std::to_string(int(motorArr[3].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title5, std::to_string(int(motorArr[4].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title6, std::to_string(int(motorArr[5].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title7, std::to_string(int(motorArr[6].get_torque())).substr(0,4).c_str());
    lv_label_set_text(title8, std::to_string(int(motorArr[7].get_torque())).substr(0,4).c_str());
  }

  if (cata_limit.get_value() == true) {
  lv_label_set_text(limitBoxLabel2, SYMBOL_OK" PRESS");
  } else {
  lv_label_set_text(limitBoxLabel2, SYMBOL_CLOSE" OPEN");
  }

lv_label_set_text(rotationBoxLabel, ("Enc Left  | angle: " + std::to_string(int(rotation_left.get_angle())).substr(0,3) + " | vel: "  + std::to_string(int(rotation_left.get_velocity())).substr(0,4)).c_str());
lv_label_set_text(rotationBoxLabel2, ("Enc Right | angle: " + std::to_string(int(rotation_right.get_angle())).substr(0,3) + " | vel: "  + std::to_string(int(rotation_right.get_velocity())).substr(0,4)).c_str());

lv_label_set_text(opticalBoxLabel2, ("Hue: " + std::to_string(int(optical.get_hue())).substr(0,3)).c_str());
lv_label_set_text(opticalBoxLabel3, ("Sat: " + std::to_string(int(optical.get_saturation())).substr(0,3)).c_str());
lv_label_set_text(opticalBoxLabel4, ("Bright: " + std::to_string(int(optical.get_brightness())).substr(0,3)).c_str());

lv_label_set_text(opticalBoxLabel5, (std::to_string(opticalCounter).substr(0,3)).c_str());

  lv_gauge_set_value(gauge1, 0, inertial.get_heading()/3.5);

lv_label_set_text(inertialBoxLabel2, ("Head: " + std::to_string(int(inertial.get_heading())).substr(0,5)).c_str());
lv_label_set_text(inertialBoxLabel3, ("Rot:  " + std::to_string(int(inertial.get_rotation())).substr(0,5)).c_str());

;

lv_label_set_text(visionBoxLabel2, ("Batt: " + std::to_string(int(pros::battery::get_voltage())).substr(0,3)).c_str());
lv_label_set_text(visionBoxLabel3, ("Curr: " + std::to_string(int(pros::battery::get_current())).substr(0,5)).c_str());
lv_label_set_text(visionBoxLabel4, ("Temp: " + std::to_string(int(pros::battery::get_temperature())).substr(0,5)).c_str());
lv_label_set_text(visionBoxLabel5, ("Cap:  " + std::to_string(int(pros::battery::get_capacity())).substr(0,5)).c_str());




    pros::delay(100);
  }

}



// REMINDER - MOTORS BUTTON CLICK ACTION
std::string temperature = "hello";



static lv_res_t btn_click_motors(lv_obj_t * btn)
{
uint8_t id = lv_obj_get_free_num(btn);
printf("Button %d is released\n", id);
/* The button is released.
* Make something here */

switch (id) {
  case 5:
    tempToggle = 0;
    break;
  case 6:
    tempToggle = 1;
    break;
  case 7:
    powerToggle = 0;
    break;
  case 8:
    powerToggle = 1;
    break;
  case 9:
    voltToggle = 0;
    break;
  case 10:
    voltToggle = 1;
    break;
  default:
    break;
}

return LV_RES_OK; /*Return OK if the button is not deleted*/
}




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
//     lv_obj_t* obj = lv_obj_create(lv_scr_act(), NULL);
// lv_obj_set_size(obj, 480, 240);
// // lv_obj_set_style(obj, &lv_style_transp); // make the container invisible
// lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);

  // Print our branding over your terminal :D
  // ez::print_ez_template();
  motorMap.insert(std::make_pair("cata", cata));
	motorMap.insert(std::make_pair("intake", intake));
	motorMap.insert(std::make_pair("driveFR", driveFR));
	motorMap.insert(std::make_pair("driveMR", driveMR));
	motorMap.insert(std::make_pair("driveBR", driveBR));
	motorMap.insert(std::make_pair("driveFL", driveFL));
	motorMap.insert(std::make_pair("driveML", driveML));
	motorMap.insert(std::make_pair("driveBL", driveBL));


  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // // Autonomous Selector using LLEMU
  // ez::as::auton_selector.add_autons({
  //   Auton("Example Drive\n\nDrive forward and come back.", drive_example),
  //   Auton("Example Turn\n\nTurn 3 times.", turn_example),
  //   Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
  //   Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
  //   Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
  //   Auton("Combine all 3 movements", combining_movements),
  //   Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  // });

  // Initialize chassis and auton selector
  chassis.initialize(true);
  // ez::as::initialize(); 







}




/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
// Gif gif("/usd/smile2.gif", lv_scr_act());

  // . . .
}





/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
    
// // INTRO GIF
// Gif gif("/usd/joyboy2.gif", lv_scr_act());
// pros::delay(3500);
// gif.clean();




// HOME PAGE


    tabview = lv_tabview_create(lv_scr_act(), NULL);
        // Create the tab view
lv_obj_set_style(tabview, &lv_style_transp); // make the container invisible
lv_tabview_set_sliding(tabview, false);


/*Create a drop down list*/
lv_obj_t *ddlpages = lv_ddlist_create(lv_scr_act(), NULL);
lv_ddlist_set_options(ddlpages,SYMBOL_HOME " AUTONS\n" 
SYMBOL_SETTINGS " MOTORS\n" 
SYMBOL_LOOP " SENSORS" );
// lv_obj_set_free_num(ddlpages, 1); /*Set a unique ID*/
    lv_ddlist_set_selected(ddlpages, 0);

lv_ddlist_set_action(ddlpages, ddlist_action); /*Set a function to call when anew option is chosen*/

lv_tabview_set_sliding(ddlpages, false);
lv_tabview_set_btns_hidden(tabview, true);
lv_obj_align(tabview, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);



lv_obj_align(ddlpages, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 15);
// lv_ddlist_set_anim_time(lv_obj_t * ddlist, uint16_t anim_time);
// void lv_ddlist_set_style(lv_obj_t *ddlist, lv_ddlist_style_t type, lv_style_t *style);
lv_ddlist_set_draw_arrow(ddlpages, true); // dropdown arrow
// lv_ddlist_set_hor_fit(ddlpages, true); // autofit content horizontally
lv_ddlist_set_hor_fit(ddlpages, false); // automatically set to true by defeault, can't manipulate obj width if this is set to true
lv_ddlist_set_align(ddlpages, LV_LABEL_ALIGN_LEFT); // align labels in ddlist, LV_LABEL_ALIGN_CENTER/LEFT/RIGHT
lv_obj_set_width(ddlpages, 175);





   


    // Create the pages
    lv_obj_t *page1 = lv_tabview_add_tab(tabview, "");
    // lv_obj_t *page2 = lv_tabview_add_tab(tabview, "");
    page2 = lv_tabview_add_tab(tabview, "");

    lv_obj_t *page3 = lv_tabview_add_tab(tabview, "");

    lv_page_set_sb_mode(page1, LV_SB_MODE_OFF);
    lv_page_set_sb_mode(page2, LV_SB_MODE_OFF);
    lv_page_set_sb_mode(page3, LV_SB_MODE_OFF);

    lv_page_set_scroll_propagation(page1, false);
    lv_page_set_scroll_propagation(page2, false);
    lv_page_set_scroll_propagation(page3, false);



/*Create a title label*/
lv_obj_t * label = lv_label_create(page1, NULL);
lv_obj_t * label2 = lv_label_create(page1, NULL);
lv_obj_t * label3 = lv_label_create(page1, NULL);
lv_obj_t * label4 = lv_label_create(page1, NULL);

lv_label_set_text(label, "");
lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

lv_label_set_text(label2, "");
lv_obj_align(label2, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

lv_label_set_text(label3, "");
lv_obj_align(label3, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

lv_label_set_text(label4, "");
lv_obj_align(label4, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

/*Create a normal button*/
lv_obj_t * btn1 = lv_btn_create(page1, NULL);
lv_obj_t * btn2 = lv_btn_create(page1, NULL);
lv_obj_t * btn3 = lv_btn_create(page1, NULL);
lv_obj_t * btn4 = lv_btn_create(page1, NULL);

// lv_cont_set_fit(btn1, true, true); /*Enable resizing horizontally and vertically*/
lv_obj_set_free_num(btn1, 1); /*Set a unique number for the button*/
lv_obj_set_free_num(btn2, 2); /*Set a unique number for the button*/
lv_obj_set_free_num(btn3, 3); /*Set a unique number for the button*/
lv_obj_set_free_num(btn4, 4); /*Set a unique number for the button*/


lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action);
lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, btn_click_action);
lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, btn_click_action);
lv_btn_set_action(btn4, LV_BTN_ACTION_CLICK, btn_click_action);

/*Add a label to the button*/
label = lv_label_create(btn1, NULL);
lv_label_set_text(label, "Skills");

label2 = lv_label_create(btn2, NULL);
lv_label_set_text(label2, "Right Elims");

label3 = lv_label_create(btn3, NULL);
lv_label_set_text(label3, "Left Elims");

label4 = lv_label_create(btn4, NULL);
lv_label_set_text(label4, "Solo WP");



lv_obj_set_size(btn1, 175, 35);
lv_obj_set_size(btn2, 175, 35);
lv_obj_set_size(btn3, 175, 35);
lv_obj_set_size(btn4, 175, 35);

lv_obj_align(btn1, NULL, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

lv_obj_align(btn2, btn1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
lv_obj_align(btn3, btn2, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
lv_obj_align(btn4, btn3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);




// NOTE - BRAIN SCREEN IS ROUGHLY 480x240 PX






// GIF
lv_obj_t* gifObjHome = lv_obj_create(page1, NULL);

lv_obj_set_size(gifObjHome, 275, 75);
lv_obj_set_style(gifObjHome, &lv_style_transp); // make the container invisible
lv_obj_align(gifObjHome, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 10);

static Gif  gifhome("/usd/arsenalrainbow.gif", gifObjHome);




/*Create a scroll bar style*/
static lv_style_t style_sb;
lv_style_copy(&style_sb, &lv_style_plain);
style_sb.body.main_color = LV_COLOR_BLACK;
style_sb.body.grad_color = LV_COLOR_BLACK;
style_sb.body.border.color = LV_COLOR_WHITE;
style_sb.body.border.width = 1;
style_sb.body.border.opa = LV_OPA_70;
style_sb.body.radius = LV_RADIUS_CIRCLE;
style_sb.body.opa = LV_OPA_60;
// style_sb.body.padding.hor = 50;
/*Create a normal Text area*/
ta1 = lv_ta_create(page1, NULL);
lv_obj_set_size(ta1, 275, 135);
lv_obj_align(ta1, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, -10);

lv_ta_set_style(ta1,LV_TA_STYLE_SB, &style_sb); /*Apply the scroll bar style*/
    lv_ta_set_text(ta1, (autoName + "\n" + autoDesc).c_str()); /*Set an initial text*/


//  TODO DISABLE SCROLLING ON PAGE ONE












  // MOTORS PAGE







/*Create a title label*/
lv_obj_t * label_temp = lv_label_create(page2, NULL);
lv_label_set_text(label_temp, "Default buttons");
lv_obj_align(label_temp, NULL, LV_ALIGN_IN_LEFT_MID, 0, 5);



static lv_style_t rel_style, pr_style;
lv_style_copy(&rel_style, &lv_style_btn_rel);
rel_style.body.radius = 0;
rel_style.body.main_color = LV_COLOR_GREEN;





button1 = lv_btn_create(page2, NULL);
lv_obj_set_size(button1, 90, 50);
lv_obj_align(button1, NULL, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
lv_btn_set_action(button1, LV_BTN_ACTION_CLICK, btn_click_motors);

button2 = lv_btn_create(page2, button1);
lv_obj_align(button2, button1, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
lv_btn_set_action(button2, LV_BTN_ACTION_CLICK, btn_click_motors);


lv_btn_set_style(button1, LV_BTN_STYLE_REL, &rel_style);
lv_btn_set_style(button2, LV_BTN_STYLE_REL, &rel_style);

/*Add a label to the button*/
label = lv_label_create(button1, NULL);
lv_label_set_text(label, "temp");

/*Add a label to the button*/
label = lv_label_create(button2, NULL);
lv_label_set_text(label, "current");



button3 = lv_btn_create(page2, NULL);
lv_obj_set_size(button3, 90, 50);
lv_obj_align(button3, NULL, LV_ALIGN_OUT_BOTTOM_LEFT, 0, -6);
lv_btn_set_action(button3, LV_BTN_ACTION_CLICK, btn_click_motors);

button4 = lv_btn_create(page2, button3);
lv_obj_align(button4, button3, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
lv_btn_set_action(button4, LV_BTN_ACTION_CLICK, btn_click_motors);


lv_btn_set_style(button3, LV_BTN_STYLE_REL, &rel_style);
lv_btn_set_style(button4, LV_BTN_STYLE_REL, &rel_style);

/*Add a label to the button*/
label = lv_label_create(button3, NULL);
lv_label_set_text(label, "power");

/*Add a label to the button*/
label = lv_label_create(button4, NULL);
lv_label_set_text(label, "velocity");



button5 = lv_btn_create(page2, NULL);
lv_obj_set_size(button5, 90, 50);
lv_obj_align(button5, NULL, LV_ALIGN_OUT_BOTTOM_LEFT, 0, -6);
lv_btn_set_action(button5, LV_BTN_ACTION_CLICK, btn_click_motors);

button6= lv_btn_create(page2, button3);
lv_obj_align(button6, button5, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
lv_btn_set_action(button6, LV_BTN_ACTION_CLICK, btn_click_motors);


lv_btn_set_style(button5, LV_BTN_STYLE_REL, &rel_style);
lv_btn_set_style(button6, LV_BTN_STYLE_REL, &rel_style);

/*Add a label to the button*/
label = lv_label_create(button5, NULL);
lv_label_set_text(label, "volts");

/*Add a label to the button*/
label = lv_label_create(button6, NULL);
lv_label_set_text(label, "torque");

lv_obj_set_free_num(button1, 5); /*Set a unique number for the button*/
lv_obj_set_free_num(button2, 6); 
lv_obj_set_free_num(button3, 7); /*Set a unique number for the button*/
lv_obj_set_free_num(button4, 8); /*Set a unique number for the button*/
lv_obj_set_free_num(button5, 9); /*Set a unique number for the button*/
lv_obj_set_free_num(button6, 10); /*Set a unique number for the button*/




// BIG DASHBOARD
list1 = lv_list_create(page2, NULL);

lv_obj_set_size(list1, 275, 225);
lv_obj_align(list1, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 7);

lv_list_add(list1, NULL, ("Cata " + std::to_string(cata.get_port())).c_str(), list_release_action); // no symbols, null instead
lv_list_add(list1, NULL, ("Intake " + std::to_string(intake.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtFR " + std::to_string(driveFR.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtMR " + std::to_string(driveMR.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtBR " + std::to_string(driveBR.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtFL " + std::to_string(driveFL.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtML " + std::to_string(driveML.get_port())).c_str(), list_release_action);
lv_list_add(list1, NULL, ("dtBL " + std::to_string(driveBL.get_port())).c_str(), list_release_action);

// maybe make symbol combo change  based on what is selected

static lv_style_t dashboardStyle;
lv_style_copy(&dashboardStyle, &lv_style_btn_rel);

// dashboardStyle.text.font = &lv_font_symbol_30;
dashboardStyle.body.padding.ver = 4; // changing height of item
dashboardStyle.body.padding.hor = 10; // changing height of item
// dashboardStyle.body.padding.inner = 100;

dashboardStyle.body.radius = 0;


lv_list_set_style(list1, LV_LIST_STYLE_BTN_REL, &dashboardStyle);

	 pros::Task dashboard_update_task (dashboardTask, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "Updating motor values");

      // for (int k = 0; k<8; k++) {
      //   std::string  labelName = "title " + std::to_string(k);
      //   lv_obj_t * title = lv_label_create(page2, NULL);
      //   lv_label_set_text(title, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
      //   lv_obj_align(title, list1, LV_ALIGN_IN_TOP_RIGHT, -5, listSpacerCounter); /*Align to the top*/
      //   listSpacerCounter += 28;
      // }


      // 1st column
        title = lv_label_create(page2, NULL);
        lv_label_set_text(title, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        title2 = lv_label_create(page2, NULL);
        lv_label_set_text(title2, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title2, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title3 = lv_label_create(page2, NULL);
        lv_label_set_text(title3, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title3, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;


        title4 = lv_label_create(page2, NULL);
        lv_label_set_text(title4, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title4, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title5 = lv_label_create(page2, NULL);
        lv_label_set_text(title5, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title5, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title6 = lv_label_create(page2, NULL);
        lv_label_set_text(title6, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title6, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title7 = lv_label_create(page2, NULL);
        lv_label_set_text(title7, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title7, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title8 = lv_label_create(page2, NULL);
        lv_label_set_text(title8, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title8, list1, LV_ALIGN_IN_TOP_RIGHT, -15, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;





      // 2nd column
      listSpacerCounter = 5;
      int spacerMid = -75;
        title9 = lv_label_create(page2, NULL);
        lv_label_set_text(title9, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title9, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        title10 = lv_label_create(page2, NULL);
        lv_label_set_text(title10, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title10, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title11 = lv_label_create(page2, NULL);
        lv_label_set_text(title11, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title11, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;


        title12 = lv_label_create(page2, NULL);
        lv_label_set_text(title12, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title12, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title13 = lv_label_create(page2, NULL);
        lv_label_set_text(title13, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title13, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title14 = lv_label_create(page2, NULL);
        lv_label_set_text(title14, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title14, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title15 = lv_label_create(page2, NULL);
        lv_label_set_text(title15, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title15, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title16 = lv_label_create(page2, NULL);
        lv_label_set_text(title16, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title16, list1, LV_ALIGN_IN_TOP_RIGHT, spacerMid, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;




  // 3rd column
      listSpacerCounter = 5;
      int spacerLeft = -135;

          title17 = lv_label_create(page2, NULL);
        lv_label_set_text(title17, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title17, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        title18 = lv_label_create(page2, NULL);
        lv_label_set_text(title18, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title18, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title19 = lv_label_create(page2, NULL);
        lv_label_set_text(title19, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title19, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;


        title20 = lv_label_create(page2, NULL);
        lv_label_set_text(title20, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title20, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title21 = lv_label_create(page2, NULL);
        lv_label_set_text(title21, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title21, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title22 = lv_label_create(page2, NULL);
        lv_label_set_text(title22, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title22, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title23 = lv_label_create(page2, NULL);
        lv_label_set_text(title23, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title23, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;

        
        title24 = lv_label_create(page2, NULL);
        lv_label_set_text(title24, temperature.c_str()); // horrible line of code: get_temp goes from double to int to string to substring to c_str
        lv_obj_align(title24, list1, LV_ALIGN_IN_TOP_RIGHT, spacerLeft, listSpacerCounter); /*Align to the top*/
        listSpacerCounter += 28;









lv_obj_t * box1;
box1 = lv_cont_create(page2, NULL);
lv_cont_set_fit(box1, false, false);
lv_obj_align(box1, title, LV_ALIGN_CENTER, 35, 27);
lv_obj_set_size(box1, 60, 223);

lv_obj_t * box2;
box2 = lv_cont_create(page2, NULL);
lv_cont_set_fit(box2, false, false);
lv_obj_align(box2, title, LV_ALIGN_CENTER, -25, 27);
lv_obj_set_size(box2, 60, 223);

lv_obj_t * box3;
box3 = lv_cont_create(page2, NULL);
lv_cont_set_fit(box3, false, false);
lv_obj_align(box3, title, LV_ALIGN_CENTER, -85, 27);
lv_obj_set_size(box3, 60, 223);



        static lv_style_t styleBG;
        lv_style_copy(&styleBG, &lv_style_plain);
        styleBG.body.main_color = LV_COLOR_YELLOW; 
        styleBG.body.grad_color = LV_COLOR_YELLOW; 
        styleBG.body.opa = 100;
        
        lv_cont_set_style(box1, &styleBG);

        static lv_style_t styleBG2;
        lv_style_copy(&styleBG2, &lv_style_plain);
        styleBG2.body.main_color = LV_COLOR_BLUE; 
        styleBG2.body.grad_color = LV_COLOR_BLUE; 
        styleBG2.body.opa = 100;
        
        lv_cont_set_style(box2, &styleBG2);


                static lv_style_t styleBG3;
        lv_style_copy(&styleBG3, &lv_style_plain);
        styleBG3.body.main_color = LV_COLOR_RED; 
        styleBG3.body.grad_color = LV_COLOR_RED; 
        styleBG3.body.opa = 100;
        
        lv_cont_set_style(box3, &styleBG3);









    // SENSORS PAGE




/*Create a title label*/
lv_obj_t * label6 = lv_label_create(page3, NULL); lv_obj_t * label7 = lv_label_create(page3, NULL); lv_obj_t * label8 = lv_label_create(page3, NULL); lv_obj_t * label9 = lv_label_create(page3, NULL); lv_label_set_text(label6, ""); lv_obj_align(label6, NULL, LV_ALIGN_OUT_TOP_LEFT, 0, 5); lv_label_set_text(label7, ""); lv_obj_align(label7, NULL, LV_ALIGN_OUT_TOP_LEFT, 0, 5); lv_label_set_text(label8, ""); lv_obj_align(label8, NULL, LV_ALIGN_OUT_TOP_LEFT, 0, 5); lv_label_set_text(label9, ""); lv_obj_align(label9, NULL, LV_ALIGN_OUT_TOP_LEFT, 0, 5); /*Create a normal button*/
lv_obj_t * btn111 = lv_btn_create(page3, NULL); lv_obj_t * btn222 = lv_btn_create(page3, NULL); lv_obj_t * btn333 = lv_btn_create(page3, NULL); lv_obj_t * btn444 = lv_btn_create(page3, NULL); // lv_cont_set_fit(btn1, true, true); /*Enable resizing horizontally and vertically*/ lv_obj_set_free_num(btn111, 1); /*Set a unique number for the button*/ lv_obj_set_free_num(btn222, 2); /*Set a unique number for the button*/ lv_obj_set_free_num(btn333, 3); /*Set a unique number for the button*/ lv_obj_set_free_num(btn444, 4); /*Set a unique number for the button*/ lv_btn_set_action(btn111, LV_BTN_ACTION_CLICK, btn_click_action); lv_btn_set_action(btn222, LV_BTN_ACTION_CLICK, btn_click_action); lv_btn_set_action(btn333, LV_BTN_ACTION_CLICK, btn_click_action); lv_btn_set_action(btn444, LV_BTN_ACTION_CLICK, btn_click_action);
/*Add a label to the button*/ label6 = lv_label_create(btn111, NULL); lv_label_set_text(label6, "Skills"); label7 = lv_label_create(btn222, NULL); lv_label_set_text(label7, "Right Elims"); label8 = lv_label_create(btn333, NULL); lv_label_set_text(label8, "Left Elims");
label9 = lv_label_create(btn444, NULL); lv_label_set_text(label9, "Solo WP"); lv_obj_set_size(btn111, 175, 35); lv_obj_set_size(btn222, 175, 35); lv_obj_set_size(btn333, 175, 35); lv_obj_set_size(btn444, 175, 35);
lv_obj_align(btn111, NULL, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 250); lv_obj_align(btn222, btn111, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5); lv_obj_align(btn333, btn222, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5); lv_obj_align(btn444, btn333, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5); lv_btn_set_style(btn111, LV_BTN_STYLE_REL, &lv_style_transp);




lv_obj_t * boxLimitContainer;
boxLimitContainer = lv_obj_create(page3, NULL);
lv_obj_set_size(boxLimitContainer, 100, 60);
lv_obj_align(boxLimitContainer, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);


lv_obj_t *limitBoxLabel;
limitBoxLabel = lv_label_create(page3, NULL);
lv_label_set_text(limitBoxLabel, "Limit 'A'");
lv_obj_align(limitBoxLabel, boxLimitContainer, LV_ALIGN_IN_TOP_MID, 0, 7);


limitBoxLabel2 = lv_label_create(page3, NULL);
lv_label_set_text(limitBoxLabel2, "hello");
lv_obj_align(limitBoxLabel2, boxLimitContainer, LV_ALIGN_IN_BOTTOM_MID, -20, -7);






// optical sensor
lv_obj_t * boxOpticalContainer;
boxOpticalContainer = lv_obj_create(page3, NULL);
lv_obj_set_size(boxOpticalContainer, 170, 120);
lv_obj_align(boxOpticalContainer, boxLimitContainer, LV_ALIGN_OUT_RIGHT_MID, 5, 30);

lv_obj_t *opticalBoxLabel;
opticalBoxLabel = lv_label_create(page3, NULL);
lv_label_set_text(opticalBoxLabel, "Optical 15");
lv_obj_align(opticalBoxLabel, boxOpticalContainer, LV_ALIGN_IN_TOP_LEFT, 7, 5);

opticalBoxLabel2 = lv_label_create(page3, NULL);
lv_label_set_text(opticalBoxLabel2, "Hue");
lv_obj_align(opticalBoxLabel2, boxOpticalContainer, LV_ALIGN_IN_TOP_LEFT, 7, 40);

opticalBoxLabel3 = lv_label_create(page3, NULL);
lv_label_set_text(opticalBoxLabel3, "Sat");
lv_obj_align(opticalBoxLabel3, boxOpticalContainer, LV_ALIGN_IN_TOP_LEFT, 7, 65);

opticalBoxLabel4 = lv_label_create(page3, NULL);
lv_label_set_text(opticalBoxLabel4, "Bright");
lv_obj_align(opticalBoxLabel4, boxOpticalContainer, LV_ALIGN_IN_TOP_LEFT, 7, 90);

lv_obj_t * boxOpticalContainer2;
boxOpticalContainer2 = lv_obj_create(page3, NULL);
lv_obj_set_size(boxOpticalContainer2, 70, 120);
lv_obj_align(boxOpticalContainer2, boxOpticalContainer, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

lv_obj_t *upBtn;
upBtn = lv_btn_create(boxOpticalContainer2, NULL);
lv_obj_set_free_num(upBtn, 111);
lv_btn_set_action(upBtn, LV_BTN_ACTION_CLICK, btn_click_optical);
lv_obj_align(upBtn, boxOpticalContainer2, LV_ALIGN_IN_TOP_MID, 38, 10);
lv_obj_set_size(upBtn, 50, 30);

opticalBoxLabel5 = lv_label_create(page3, NULL);
lv_label_set_text(opticalBoxLabel5, "Bright");
lv_obj_align(opticalBoxLabel5, boxOpticalContainer2, LV_ALIGN_CENTER, 18, 0);

lv_obj_t *downBtn;
downBtn = lv_btn_create(boxOpticalContainer2, NULL);
lv_obj_set_free_num(downBtn, 222);
lv_btn_set_action(downBtn, LV_BTN_ACTION_CLICK, btn_click_optical);
lv_obj_align(downBtn, boxOpticalContainer2, LV_ALIGN_IN_BOTTOM_MID, 38, 40);
lv_obj_set_size(downBtn, 50, 30);






lv_obj_t * boxRotationContainer;
boxRotationContainer = lv_obj_create(page3, NULL);
lv_obj_set_size(boxRotationContainer, 280, 55);
lv_obj_align(boxRotationContainer, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 85);


rotationBoxLabel = lv_label_create(page3, NULL);
lv_label_set_text(rotationBoxLabel, "Enc Left");
lv_obj_align(rotationBoxLabel, boxRotationContainer, LV_ALIGN_IN_TOP_LEFT, 7, 5);


rotationBoxLabel2 = lv_label_create(page3, NULL);
lv_label_set_text(rotationBoxLabel2, "Enc Right");
lv_obj_align(rotationBoxLabel2, boxRotationContainer, LV_ALIGN_IN_BOTTOM_LEFT, 7, -5);





lv_obj_t * boxInertialContainer;
boxInertialContainer = lv_obj_create(page3, NULL);
lv_obj_set_size(boxInertialContainer, 235, 100);
lv_obj_align(boxInertialContainer, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 145);

lv_obj_t *inertialBoxLabel;
inertialBoxLabel = lv_label_create(page3, NULL);
lv_label_set_text(inertialBoxLabel, "Inertial 14");
lv_obj_align(inertialBoxLabel, boxInertialContainer, LV_ALIGN_IN_TOP_LEFT, 12, 10);

static lv_color_t needle_colors[] = {LV_COLOR_BLUE, LV_COLOR_ORANGE, LV_COLOR_PURPLE};


gauge1 = lv_gauge_create(page3, NULL);
lv_obj_set_size(gauge1, 90, 90);

lv_gauge_set_needle_count(gauge1, 1, needle_colors);
lv_obj_align(gauge1, NULL, LV_ALIGN_IN_TOP_MID, -55, 150);



lv_gauge_set_scale(gauge1, 360, 36, 0);
lv_gauge_set_value(gauge1, 0, 180);

inertialBoxLabel2 = lv_label_create(page3, NULL);
lv_label_set_text(inertialBoxLabel2, "Heading");
lv_obj_align(inertialBoxLabel2, boxInertialContainer, LV_ALIGN_IN_TOP_LEFT, 35, 40);


inertialBoxLabel3 = lv_label_create(page3, NULL);
lv_label_set_text(inertialBoxLabel3, "Rot");
lv_obj_align(inertialBoxLabel3, boxInertialContainer, LV_ALIGN_IN_BOTTOM_LEFT, 35, -15);




lv_obj_t * boxVisionContainer;
boxVisionContainer = lv_obj_create(page3, NULL);
lv_obj_set_size(boxVisionContainer, 220, 100);
lv_obj_align(boxVisionContainer, NULL, LV_ALIGN_IN_TOP_LEFT, 240, 145);


lv_obj_t *visionBoxLabel;
visionBoxLabel = lv_label_create(page3, NULL);
lv_label_set_text(visionBoxLabel, "Battery");
lv_obj_align(visionBoxLabel, boxVisionContainer, LV_ALIGN_IN_TOP_MID, 0, 8);

visionBoxLabel2 = lv_label_create(page3, NULL);
lv_label_set_text(visionBoxLabel2, "Sig ID");
lv_obj_align(visionBoxLabel2, boxVisionContainer, LV_ALIGN_IN_TOP_LEFT, 20, 40);


visionBoxLabel3 = lv_label_create(page3, NULL);
lv_label_set_text(visionBoxLabel3, "Sig Size");
lv_obj_align(visionBoxLabel3, boxVisionContainer, LV_ALIGN_IN_BOTTOM_LEFT, 20, -15);

visionBoxLabel4 = lv_label_create(page3, NULL);
lv_label_set_text(visionBoxLabel4, "Obj Count");
lv_obj_align(visionBoxLabel4, boxVisionContainer, LV_ALIGN_IN_TOP_RIGHT, -20, 40);

visionBoxLabel5 = lv_label_create(page3, NULL);
lv_label_set_text(visionBoxLabel5, "Mid Coord");
lv_obj_align(visionBoxLabel5, boxVisionContainer, LV_ALIGN_IN_BOTTOM_RIGHT, -20, -15);



        static lv_style_t styleBG5;
        lv_style_copy(&styleBG5, &lv_style_plain);
        styleBG5.body.main_color = LV_COLOR_BLUE; 
        styleBG5.body.grad_color = LV_COLOR_BLUE; 
        styleBG5.body.opa = 100;
        

        lv_cont_set_style(boxLimitContainer, &styleBG5);
        lv_cont_set_style(boxOpticalContainer, &styleBG5);
        lv_cont_set_style(boxRotationContainer, &styleBG5);
        lv_cont_set_style(boxInertialContainer, &styleBG5);
        lv_cont_set_style(boxVisionContainer, &styleBG5);




}



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
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.

  switch (autoCounter) {
    case 1:
      skills();
      break;
    case 2:
      right_elims();
      break;
    case 3:
      left_elims();
      break;
    case 4: 
      solo_WP();
      break;
    default:
      break;
  }

}





bool swingEnabled = false;

void swingTask(void *param) {
  while (true) {

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      swingEnabled = true;

      chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
      chassis.wait_drive();

      swingEnabled = false;
    } else if (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !swingEnabled) {
       chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    }
  pros::delay(150);
  }
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
  // This is preference to what you like to drive on.
  // chassis.set_drive_brake(MOTOR_BRAKE_COAST);









  // DISCONNECT DETECTING TASK
	pros::Task disconnect_detection_task (disconnect_detection, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Detect Disconnect");
  
  // SWING TURNING IN DRIVER TASK
  pros::Task swing_skills_task (swingTask, (void*)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Swing turn driver task");



  while (true) {

    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intake = 127;
    cata = -127;

  } else {
    intake = 0;
    cata = 0;
  }

  // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
  //   inertial.set_heading(0);
  //   inertial.set_rotation(0);
  //   inertial.set_yaw(0);

  //   chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
  //   chassis.wait_drive();
  // } else {
  //   chassis.arcade_standard(ez::SPLIT); // Standard split arcade

  // }
    

  
    pros::delay(20);
  }
}
