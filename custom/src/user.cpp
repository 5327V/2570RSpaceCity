#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/intake.h"
// Modify autonomous, driver, or pre-auton code below
int auton_selected = 0;
bool auto_started = false;

void runAutonomous() {
  auto_started = true;
  switch(auton_selected) {
    case 0:
      awp();
      break;
    case 1:
      left9Long();
      break;  
    case 2:
      right9Long();
      break;
    case 3:
      left7LongandWing();
      break; 
    case 4:
      //right7LongandWing();
      break;
    case 5:
      leftLongAndMid();
      break;
    case 6:
      rightLongAndLow();
    case 7:
      left9LongDisrupt();
    case 8:
      leftLongAndMidDisrupt();

    case 9:
      right9LongDisrupt();
  }
}

int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  bool downPressed;
  bool bPressed;
  
  while (true) {
    antiJamTask();
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.12 + ch1 * 0.12 , ch3 * 0.12 - ch1 * 0.12);

    if(r1){
      storeIntake();
      middleGoal.set(false);
    } else if(r2){
      scoreLongGoal();
      middleGoal.set(false);
    } else if(l1){
      middleGoal.set(true);
      scoreMiddleGoal();
    } else if(l2){
      leftWing.set(false);
      middleGoal.set(false);
      
    } else if(button_b){
      outtake();
    }
    else{
      stopIntake();
      leftWing.set(true);
      middleGoal.set(false);
    }
    if(controller_1.ButtonDown.PRESSED){
      downPressed = !downPressed;
      if(downPressed){
        matchloader.set(true);
      } else{
        matchloader.set(false);
      }
    }

    

    wait(10, msec); 
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }
  controller_1.rumble("..--");
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Selected Auton:");
    switch(auton_selected){
      case 0:
        Brain.Screen.printAt(5, 40, "AWP");
        break;
      case 1:
        Brain.Screen.printAt(5, 40, "Left 9 Long");
        break;
      case 2:
        Brain.Screen.printAt(5, 40, "Right 9 Long");
        break;
      case 3:
        Brain.Screen.printAt(5, 40, "Left 7 & Wing");
        break;
      case 4:
        Brain.Screen.printAt(5, 40, "Right 7 & Wing");
        break;
      case 5:
        Brain.Screen.printAt(5, 40, "Left Long + Mid");
        break;
      case 6:
        Brain.Screen.printAt(5, 40, "Right Long + Low");
        break;
      case 7:
        Brain.Screen.printAt(5, 40, "Left 9 & Disrupt");
        break;
      case 8:
        Brain.Screen.printAt(5, 40, "Left Long & Mid + Disrupt");
        break;
      case 9:
        Brain.Screen.printAt(5, 40, "Right 9 & Disrupt");
        break;
    }
  }
}