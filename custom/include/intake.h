#pragma once


extern double intakeSpeed;
extern double hoodSpeed;

extern void scoreLongGoal();
extern void scoreMiddleGoal();
extern void outtake();
extern void stopIntake();
extern void storeIntake();
extern void antiJamTask();
extern void manualIntake(double speed, double hoodSpd);
extern bool isRed;
extern bool rejectBall;