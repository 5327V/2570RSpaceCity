#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"
#include "../custom/include/intake.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

void left9Long(){
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, false, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(-30, 34, 1, -39, 0.1, 3000, true, 6);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(-23, 15, -1, 3000, false, 8);
  stopIntake();
  boomerang(-32, 15, -1,90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-14, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1300, msec);
  driveChassis(0,0);
  resetOdom(-31.5, 15);
  matchloader.set(true);
  moveToPoint(-31, -7, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1050, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-31, 10, -1, 3000, false, 6);
  driveChassis(-6, -6);
  vex::wait(1100, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);
}

void left9LongDisrupt(){
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-6, 24, 1, 2000, false, 6);
  boomerang(-30, 33.5, 1, -39, 0.1, 3000, true, 9);
  matchloader.set(true);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(-23, 15, -1, 3000, false, 8);
  boomerang(-32, 15, -1,90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-14, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1100, msec);
  driveChassis(0,0);
  resetOdom(-31.5, 15);
  matchloader.set(true);
  moveToPoint(-30, -7, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(950, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-31, 10, -1, 3000, false, 6);
  driveChassis(-6, -6);
  vex::wait(1000, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);
}


void right9Long(){
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(6, 24, 1, 2000, false, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(1000, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(33, 43, 1, 40, 0.1, 3000, true, 5);
  matchloader.set(true);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(23, 13, -1, 3000, false, 8);
  stopIntake();
  boomerang(45, 13, -1,-90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-12.2, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1500, msec);
  matchloader.set(true);
  moveToPoint(45, 6, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1000, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(45.5, 10, -1, 3000, false, 6);
  driveToHeading(-16, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);

}

void right9LongDisrupt(){
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = 45;
  vex::task matchloadDeploy([]{
    vex::wait(800, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(6, 24, 1, 2000, false, 5);
  matchloader.set(true);
  vex::task matchloadDeploy2([]{
    vex::wait(1000, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(32, 45, 1, 40, 0.1, 3000, true, 12);
  matchloader.set(true);
  vex::wait(80, msec);
  heading_correction_kp = 0.3;
  moveToPoint(28, 13, -1, 3000, false, 8);
  stopIntake();
  boomerang(48, 13, -1,-90, 0.1, 3000, true, 8);
  turnToAngle(180, 1000, true, 7);
  driveToHeading(-14, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1500, msec);
  matchloader.set(true);
  moveToPoint(48.5, 6, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(1000, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(49, 10, -1, 3000, false, 6);
  driveToHeading(-20, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(12, 180, 3000, true, 6);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 6);

}

void leftLongAndMid(){
  leftWing.set(true);
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, false, 5);
  matchloader.set(false);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    //matchloader.set(true);
    return 0;
  });
  boomerang(-24, 33, 1, -44, 0.1, 3000, true, 5);
  driveChassis(4,4);
  vex::wait(100,msec);
  //matchloader.set(true);
  vex::wait(80, msec);
  moveToPoint(-0.5, 23, -1, 3000, false, 5);
  driveChassis(-1,-1);
  vex::wait(10, msec);
  //matchloader.set(false);
  turnToAngle(-138, 1000, true, 5);
  driveTo(-10,2000, true, 5);
  middleGoal.set(true);
  scoreMiddleGoal();
  vex::wait(500, msec);
  stopIntake();
  middleGoal.set(false);
  matchloader.set(true);
  moveToPoint(-22.8, 1, 1, 3000, false, 6);
  turnToAngle(180, 300, true, 6);
  driveToHeading(-20, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(950, msec);
  matchloader.set(true);
  moveToPoint(-23.5, 0, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(6,6);
  vex::wait(900, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-23, 5, -1, 3000, false, 6);
  driveToHeading(-20, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1400, msec);
  driveToHeading(10, 180, 3000, true, 6);
  driveToHeading(-20, 180, 3000, true, 6);

}

void leftLongAndMidDisrupt(){
  min_output = 100;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-7, 24, 1, 2000, false, 5);
  matchloader.set(true);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  boomerang(-30, 34, 1, -39, 0.1, 3000, true, 12);
  vex::wait(80, msec);
  moveToPoint(-0.5, 23, -1, 3000, false, 5);
  driveChassis(-1,-1);
  vex::wait(10, msec);
  turnToAngle(-138, 1000, true, 5);
  driveTo(-7,2000, true, 5);
  middleGoal.set(true);
  scoreMiddleGoal();
  vex::wait(450, msec);
  stopIntake();
  middleGoal.set(false);
  moveToPoint(-23.6, -2, 1, 3000, false, 6);
  turnToAngle(178, 300, true, 6);
  driveToHeading(-18, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(850, msec);
  matchloader.set(true);
  moveToPoint(-23.5, -7, 1, 3000, false, 6);
  storeIntake();
  turnToAngle(180, 3000);
  driveChassis(7,7);
  vex::wait(900, msec);
  vex::task scoreL([]{
    vex::wait(1100, msec);
    scoreLongGoal();
    return 0;
  });
  moveToPoint(-23, 5, -1, 3000, false, 6);
  driveToHeading(-10, 180, 3000, true, 6);
  scoreLongGoal();
  vex::wait(1500, msec);
  driveToHeading(10, 180, 3000, true, 6);
  driveToHeading(-20, 180, 3000, true, 6);

}

void rightLongAndLow(){
  leftWing.set(true);
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });


  moveToPoint(0,30, 1, 2000, true, 6);
  manualIntake(4, 0);
  vex::wait(350, msec);
  moveToPoint(0,20,-1, 2000, true, 6);
  turnToAngle(80, 800, true, 8);
  storeIntake();
  vex::task matchloadDeploy([]{
    vex::wait(400, msec);
    matchloader.set(true);
    vex::wait(200, msec);
    matchloader.set(false);
    return 0;
  });
  moveToPoint(18,30, 1, 2000, true, 6);
  vex::task matchloadDeploy2([]{
    vex::wait(850, msec);
    //matchloader.set(true);
    
    return 0;
  });
  boomerang(34,44.8, 1, 45, 0.1, 3000, true, 5);
  driveChassis(2,2);
  vex::wait(200, msec);
  driveChassis(0,0);
  matchloader.set(true);
  heading_correction_kp = 0.3;
  moveToPoint(32, 40, -1, 3000,false, 8);
  moveToPoint(32, 10, -1, 3000,false, 8);
  turnToAngle(-90, 800, false, 6);
  moveToPoint(49, 8, -1, 3000,false, 6);
  turnToAngle(180, 800, true, 8);
  vex::task scoreLongGoal1([]{
    vex::wait(600, msec);
    scoreLongGoal();
    
    return 0;
  });
  heading_correction_kp = 1;
  driveToHeading(-12, 180, 3000, true, 7);
  driveChassis(-1,-1);
  vex::wait(1400, msec);
  resetOdom(47, 18);
  moveToPoint(46.5, -2, 1, 3000,false, 6);
  storeIntake();
  turnToAngle(180, 800, true, 8);
  driveChassis(6,6);
  vex::wait(1200, msec);
  moveToPoint(47.2, 10, -1, 3000,false, 6);
  vex::task scoreLongGoal2([]{
    vex::wait(600, msec);
    scoreLongGoal();
    
    return 0;
  });
  driveToHeading(-13, 180, 3000, true, 7);
  scoreLongGoal();
  vex::wait(1300, msec);
  driveToHeading(12, 180, 3000, true, 7);
  vex::wait(20, msec);
  driveToHeading(-20, 180, 3000, true, 12);
}

void awp(){
 min_output = 100;
 vex::task antiJamF([]{
   while(1){
     antiJamTask();
     vex::wait(20, msec);
   }
   return 0;
 });
 leftWing.set(true);
 // Use this for tuning linear and turn pid
 storeIntake();
 matchloader.set(true);
 moveToPoint(0,28, 1, 2000, true, 10);
 turnToAngle(88, 800, true, 8);
 vex::wait(50, msec);
 driveChassis(5,5);
 vex::wait(800, msec);
 driveChassis(4,4);
 vex::wait(200, msec);
 vex::task scoreLongGoal1([]{
   vex::wait(950, msec);
   scoreLongGoal();
   return 0;
 });
 moveToPoint(-5,35, -1, 2000, false, 10);


 driveToHeading(-6, 90, 1000, false, 6);
 driveChassis(-4,-4);
 matchloader.set(false);
 vex::wait(500,msec);
 driveChassis(0,0);
 vex::wait(900, msec);
 resetOdom(-35,26.8);
 //resetAngle(90);
 heading_correction_kp = 1.1;
 moveToPoint(-33, 23, 1, 2000, false, 8);


 turnToAngle(-141, 1000, true, 6);
 storeIntake();
 //correct_angle = 180;
 heading_correction_kp = 0.8;
 correct_angle = normalizeTarget(-141);
 driveTo(22, 3000, false, 7);
 //resetAngle(-144);
 matchloader.set(true);
 //boomerang(-37,0, 1, -110, 0.1, 2000, true, 8);
 turnToAngle(-176, 800, false, 5);
 matchloader.set(false);
 heading_correction_kp = 1.1;
 vex::task mmiddle([]{
   vex::wait(50, msec);
   matchloader.set(true);
   vex::wait(300,msec);
   matchloader.set(false);
   vex::wait(1300, msec);
   matchloader.set(true);
   return 0;
 });
 boomerang(-38, -50, 1, -178, 0.1, 2000, true, 6);
 matchloader.set(true);
 correct_angle = 180;
 //driveTo(-0.5, 1200, false, 8);
 turnToAngle(134, 1000, true, 6);
 vex::task readyUpMiddle([]{
   vex::wait(150, msec);
   outtake();
   vex::wait(250, msec);
   stopIntake();
   vex::wait(150, msec);
   middleGoal.set(true);
   vex::wait(50, msec);
   scoreMiddleGoal();
   return 0;
 });
 driveTo(-18, 1200, true, 5);
 
 driveChassis(-2, -2);
 vex::wait(300, msec);
 stopIntake();
 matchloader.set(true);
 correct_angle = normalizeTarget(-35);
 moveToPoint(-11, -65, 1, 2000, false, 9);
 middleGoal.set(false);
 matchloader.set(true);
 storeIntake();
 turnToAngle(85, 1000, true, 7);
 matchloader.set(true);
 driveChassis(6,6);
 vex::wait(900, msec);
 driveChassis(1,1);
 vex::wait(200, msec);
 correct_angle = 88;
 vex::task reds([]{
   vex::wait(650, msec);
   outtake();
   vex::wait(50, msec);
   scoreLongGoal();
   return 0;
 });
 driveToHeading(-40, 85, 2000, false, 7);
 driveChassis(-4, -4);
}

//todo
void left7LongandWing(){
  heading_correction_kp = 0.8;
  vex::task antiJamF([]{
    while(1){
      antiJamTask();
      vex::wait(20, msec);
    }
    return 0;
  });
  // Use this for tuning linear and turn pid
  storeIntake();
  correct_angle = -45;
  vex::task matchloadDeploy([]{
    vex::wait(850, msec);
    matchloader.set(true);
    return 0;
  });
  moveToPoint(-8.2, 24, 1, 2000, false, 6);
  turnToAngle(-130, 300, false, 7);
  moveToPoint(-22, -3, 1, 2000, false, 12);
  turnToAngle(180, 800, true, 7);
  driveChassis(6,6);
  vex::wait(1.1, sec);
  moveToPoint(-23, 5, -1, 2000, false, 8);
  turnToAngle(180, 800, true, 7);
  driveChassis(-7,-7);
  vex::wait(0.4, sec);
  scoreLongGoal();
  driveChassis(0,0);
  vex::wait(1.4, sec);
  driveChassis(8,8);
  vex::wait(0.1, sec);
  moveToPoint(-13, 4, 1, 2000, false, 8);
  turnToAngle(-177, 800, true, 7);
  driveToHeading(-25, 180, 3000, 5);
  stopChassis(brakeType::hold);
  //

}

void right7LongandWing(){
  
}