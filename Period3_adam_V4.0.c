#include <kipr/wombat.h>
#include <math.h>
#include <stdio.h>

/////////////////
//  constants  //
//  //  /  //  //
//    ports    //
//  //  /  //  //
int Leftm = 3;
int Rightm = 0;
int Claw = 0;
int Arm = 1;
//  //  /  //  //
int Leftb = 1;
int Rightb = 2;
int Frontb = 0;
//  //  /  //  //
int Starts = 1;
int Downs = 2;
int Fronts = 3;
//  //  /  //  //
//    other
//  //  /  //  //
float Armlength = 8;
float Armoffsety = 2;
float Armoffsetx = 2;
//  //  /  //  //
float Botoffsetx = 4;
float Botoffsety = 0;
//  //  /  //  //
float DeltaL = 0;
float DeltaR = 0;
//  //  /  //  //
float Lv = 0;
float Rv = 0;
float AccelAvg = 0;
//  //  /  //  //
float LMM = 1;
float RMM = 0.8;
//  //  /  //  //
float WheelR = 2;
float AxelRadius = 4;
float TurnRate = 2; // AxelRadius / WheelR
float BotAngle = 0;
float Dist = 0;
float Ang = 0;
float CurrentO = 0;
float CurrentX = 0;
float CurrentY = 0;
float Ooffset = 0;
float Xoffset = 0;
float Yoffset = 0;
float Xl = 0;
float Yl = 0;
float FMS = 0;
float SMS = 0;
float xL = 0;
float xR = 0;
float yL = 0;
float yR = 0;
float XnL = 0;
float XnR = 0;
int stopThread = 0;
////////////////////

////////////////////
//  DECLARATIONS  //
////////////////////

float absolute(float);
// in (value)
// out (absolute value)
// Get the absolute value of an input
// abs() wasnt working for whatever reason idk

void CollisionCheck();
// Checks if the robot is attempting to move and failing
// WIP (need to check the Gs in the direction its not going)

void CollisionReset();
// Brings the robot to face the south wall and the west wall in order to find
// the location of the colision

void CollisionRecovery(float, float, float);
// Brings the robot to the point of collision, with an optional offset (0,0,0
// for no offset.) in (x offset, y offset, angular offset)

void AthenaDecision(float, float);
// in (left delta, right delta)
// decides which method of position/orientation calculation to preform

void AOPivot();
// calculates the change in orientation of the robot given a pivot

void APCount();
// calculates the change position given a straight movement

void APOTrig();
// calculates the change in position given complex deltas

void AOCalc(float);
// calculates the new orientation given the previous orientation and the change
// from previous orientation

void APCalc(float, float);
// calculates the new position given the previous orientation and the previous
// location, as well as the delta orientation and delta position

char condition();
// returns the current state of the robot
// out(battery, orientation, position, sensor values, wheel positions, servo
// positions)

int MOE(float, float, float);
// in (value one, value two, range)
// calculates if value one is within a certain range of value two
// out (1 if values are within range, 0 if not)

void Test();
// calculates the values needed to preform a given movement
// ie. "go straight for 3 seconds"

void ArmVertical(float, float);
// in (goal position, time)
// locks the arm on the z axis as it attempts to go from the current position to
// the goal position

float rad(float);
// in (degrees)
// converts degrees to radians
// math.h's was busted idk
// out (radians)

void Velocity();
// calculates the current velocity of each wheel

void TankSpeed(float, float, float);
// in (left speed, right speed, time)
// moves the wheels independently at a certain speed for a certain amount of
// time

void TankRotation(float, float, float);
// in (left wheel goal, right wheel goal, time)
// moves the wheels independently to a goal position in a certain amount of time

void TankDistance(float, float, float);
// in (left wheel goal, right wheel goal, time)
// moves the wheels independently to a goal position in a certain amount of time

void DriveSpeed(float, float, float);
// in (base speed, turn percent, time);
// moves the wheels together at a certain speed for a certain amount of time

void DriveRotation(float, float, float);
// in (base rotations, turn rotations, time);
// moves the wheels together to a goal position in a certain amount of time

void DriveDistance(float, float, float);
// in (base distance, turn distance, time);
// moves the wheels together to a goal position in a certain amount of time

void TankAccelerate(float, float, int);
// in (left goal, right goal, acceleration value)
// moves the wheels independently, going faster over time

void DriveAccelerate(float, float, int);
// in (base speed goal, turn speed goal, acceleration value)
// moves the wheels together, going faster over time

void SmoothTankTime(float, float, float, float, int, float);
// in (first left acceleration goal, first right acceleration goal, second left
// acceleration goal, second right acceleration goal, 	acceleration value, how
// long to move at first goal) moves from current speed to a new goal, stays at
// the goal for a certain amount of time, and then goes from that goal to the
// second goal

void SmoothDriveTime(float, float, float, float, int, float);
// in (first base acceleration goal, first turn acceleration goal, second base
// acceleration goal, second turn acceleration goal, 	acceleration value, how
// long to move at first goal) moves from current speed to a new goal, stays at
// the goal for a certain amount of time, and then goes from that goal to the
// second goal

void SetArm(float, float);
// in (arm goal, claw goal)
// sets the arm and claw to a goal position

void ChangeArm(float, float);
// in (delta arm position, delta claw position)
// changes the arm from the current position to the goal position

void SlowSetArm(float, float);
// in (goal arm position, time)
// goes to the goal arm position in a certain amount of time

int DTTW(float);
// in (Degrees)
// converts degrees to wheel ticks
// out (Ticks)

float TTDW(int);
// in (Ticks)
// converts wheel ticks to degrees
// out (Degrees)

float DTIW(float);
// in (Degrees)
// converts wheel degrees to inches
// out (Inches)

float ITDW(float);
// in (Inches)
// converts inches to wheel degrees
// out (Degrees)

float TTIW(int);
// in (Ticks)
// converts wheel ticks into inches
// out (Inches)

int ITTW(float);
// in (Inches)
// converts inches to wheel ticks
// out (Ticks)

int DTTA(float);
// in (Degrees)
// converts degrees to arm tics
// out (Tics)

float TTDA(int);
// in (Tics)
// converts arm tics to degrees
// out (Degrees)

int DTTC(float);
// in (Degrees)
// converts degrees to claw tics
// out (Tics)

float TTDC(int);
// in (Tics)
// converts claw tics to degrees
// out (Degrees)

void Wait(float);
// in (seconds)
// a pause

void Brake();
// employs braking in both wheels

void CMR();
// clears motor tick counters

void HandsOff();
// initiates wait for light program
// stop touching the robot!

void Startup();
// initiates system resets, prepares the robot to run

void Velocity();
// calculates the velocity of each wheel in ticks per 0.01 seconds

void ConstThread();
// runs a constant thread containing Velocity(), Condition(), updating at a
// regular interval.

////////////////////
///////////////////
//   MOVEMENT  ///
/////////////////

void TankSpeed(float SpeedL, float SpeedR, float Time) {
  float Lo = TTIW(gmpc(Leftm));
  float Ro = TTIW(gmpc(Rightm));

  motor(Leftm, SpeedL * LMM);
  motor(Rightm, SpeedR * RMM);
  msleep((Time * 1000) * timemult);

  float Lt = TTIW(gmpc(Leftm));
  float Rt = TTIW(gmpc(Rightm));
  float DelL = Lt - Lo;
  float DelR = Rt - Ro;
  AthenaDecision(DelL, DelR);
}

void TankRotation(float RotL, float RotR, float Time) {
  int RotLn = DTTW(RotL);
  int RotRn = DTTW(RotR);
  float SpeedL = RotLn / Time;
  float SpeedR = RotRn / Time;

  TankSpeed(SpeedL, SpeedR, Time);
}

void TankDistance(float DistL, float DistR, float Time) {
  int DistLn = ITTW(DistL);
  int DistRn = ITTW(DistR);
  float SpeedL = DistLn / Time;
  float SpeedR = DistRn / Time;

  TankSpeed(SpeedL, SpeedR, Time);
}

/////////////////

void DriveSpeed(float BaseS, float DifS, float Time) {
  float SpeedL = BaseS + DifS;
  float SpeedR = BaseS - DifS;

  TankSpeed(SpeedL, SpeedR, Time);
}

void DriveRotation(float BaseRot, float DifRot, float Time) {
  float RotL = BaseRot + DifRot;
  float RotR = BaseRot - DifRot;

  TankRotation(RotL, RotR, Time);
}

void DriveDistance(float BaseD, float DifD, float Time) {
  float DistL = BaseD + DifD;
  float DistR = BaseD - DifD;

  TankDistance(DistL, DistR, Time);
}

/////////////////
/////////////////

void TankAccelerate(float GoalSpeedL, float GoalSpeedR, int Acceleration) {
  float CurrentSpeedL = Lv / 15;
  float CurrentSpeedR = Rv / 15;
  float SpeedDifL = GoalSpeedL - CurrentSpeedL;
  float SpeedDifR = GoalSpeedR - CurrentSpeedR;
  SpeedDifL = SpeedDifL / 25;
  SpeedDifR = SpeedDifR / 25;
  float SpeedL = CurrentSpeedL;
  float SpeedR = CurrentSpeedR;
  int count = 0;
  while (count < 25) {
    TankSpeed(SpeedL, SpeedR, (Acceleration / 100));
    SpeedL += SpeedDifL;
    SpeedR += SpeedDifR;
  }
}

void DriveAccelerate(float GoalBase, float GoalDif, int Acceleration) {
  float CurrentBase = (Lv + Rv) / 2;
  float CurrentDif = CurrentBase - Rv;
  float SpeedDifB = GoalBase - CurrentBase;
  float SpeedDifD = GoalDif - CurrentDif;
  SpeedDifB = SpeedDifB / 25;
  SpeedDifD = SpeedDifD / 25;
  float SpeedB = CurrentBase;
  float SpeedD = CurrentDif;
  int count = 0;
  while (count < 25) {
    DriveSpeed(SpeedB, SpeedD, (Acceleration / 1000));
    SpeedB += SpeedDifB;
    SpeedD += SpeedDifD;
  }
}

/////////////////

void SmoothTankTime(float TopL, float TopR, float BotL, float BotR,
                    int Acceleration, float Time) {
  TankAccelerate(TopL, TopR, Acceleration);
  TankSpeed(TopL, TopR, Time);
  TankAccelerate(BotL, BotR, Acceleration);
}

void SmoothDriveTime(float TopB, float TopD, float BotB, float BotD,
                     int Acceleration, float Time) {
  DriveAccelerate(TopB, TopD, Acceleration);
  DriveSpeed(TopB, TopD, Time);
  DriveAccelerate(BotB, BotD, Acceleration);
}

/////////////////
//     ARM     //
/////////////////

void SetArm(float PosA, float PosC) {
  set_servo_position(Arm, DTTA(PosA));
  set_servo_position(Claw, DTTC(PosC));
  msleep(1);
}

void ChangeArm(float DeltaA, float DeltaC) {
  int CurA = TTDA(get_servo_position(Arm));
  int CurC = TTDC(get_servo_position(Claw));
  int PosA = CurA + DeltaA;
  int PosC = CurC + DeltaC;

  SetArm(PosA, PosC);
}

void SlowSetArm(float Pos, float Time) {
  int CurPos = get_servo_position(Arm);
  int distance = Pos - CurPos;
  int Distn = distance / 50;
  int Timen = Time / 50;
  int count = 0;
  while (count < 50) {
    ChangeArm(Distn, 0);
    msleep((Timen * 1000) - 1);
    count += 1;
  }
}

/////////////////
// CONVERSIONS //
//    wheels   //
/////////////////

int DTTW(float Degrees) {
  int Ticks = Degrees / 3;
  return Ticks;
}

float TTDW(int Ticks) {
  float Degrees = Ticks * 3;
  return Degrees;
}

float DTIW(float Degrees) {
  float Inches = Degrees / 3.14;
  return Inches;
}

float ITDW(float Inches) {
  float Degrees = Inches * 3.14;
  return Degrees;
}

float TTIW(int Ticks) {
  float Inches = Ticks * 314.413;
  return Inches;
}

int ITTW(float Inches) {
  int Ticks = Inches / 314.413;
  return Ticks;
}

/////////////////
// CONVERSIONS //
//     arms    //
/////////////////

int DTTA(float Degrees) {
  int Tics = (-49.3 * Degrees) + 570;
  return Tics;
}

float TTDA(int Tics) {
  float Degrees = (-0.02027 * Tics) + 11.554054054054;
  return Degrees;
}

int DTTC(float Degrees) {
  int Tics = (12.05 * Degrees) + 723.5;
  return Tics;
}

float TTDC(int Tics) {
  float Degrees = (0.082 * Tics) - 60;
  return Degrees;
}

/////////////////
// MISCELLANEOUS//
/////////////////

void Wait(float Time) {
  Brake();
  ao();
  disable_servos();
  Lv = 0;
  Rv = 0;
  msleep(Time * 1000);
  enable_servos();
}

void Brake() {
  freeze(Leftm);
  freeze(Rightm);
}

void CMR() {
  cmpc(Leftm);
  cmpc(Rightm);
}

void HandsOff() {
  printf("and god said...");
  wait_for_light(Starts);
  shut_down_in(119);
  printf("...let there be light");
}

void Startup() {
  if (LMM > RMM) {
    float WMM = 1 / LMM;
  }
  if (LMM < RMM) {
    float WMM = 1 / RMM;
  }

  RMM = RMM * WMM;
  LMM = LMM * WMM;

  if (LMM > RMM) {
    float timemult = (1 - RMM) + 1;
  }
  if (LMM < RMM) {
    float timemult = (1 - LMM) + 1;
  }

  Brake();
  CMR();
  set_servo_enabled(Arm, 1);
  set_servo_enabled(Claw, 1);
  SetArm(0, 0);
  console_clear();
}

/////////////////
//  COLLISION  //
/////////////////

float Accelerometer() {
  AccelAvg = (accel_x() + accel_y()) / 2;
  return AccelAvg;
}

void CollisionCheck() {
  float Acceleration = 0;
  float Movement = 0;
  int Collision = 0;
  if (MOE(Accelerometer(), 0, 0.1) == 1) {
    Acceleration = 0;
  } else {
    Acceleration = 1;
  }

  if (MOE(DeltaL, 0, 10) == 1) {
    Movement = 0;
  } else {
    Movement = 1;
  }

  if (Acceleration == 1 && Movement == 1) {
    Collision = 1;
  } else {
    Collision = 0;
  }
}

void CollisionReset() {
  // face bottom
  // CMR()
  // drive
  // back up
  // Ydist = MPC
  // face left
  // CMR()
  // drive
  // back up
  // Xdist = MPC
  // return Xdist, Ydist
}

void CollisionRecovery(float x, float y, float o) {
  // if collided
  // run reset
  // adjust values for offset
  // trig n follow values
  // turn to orrientation
}

/////////////////
// PATHFINDING //
/////////////////

/////////////////
//   ATHENA    //
/////////////////

void AthenaDecision(float DeltaL, float DeltaR) {
  if (absolute(DeltaL) == absolute(DeltaR) && DeltaL != DeltaR) {
    AOPivot();
  }
  if (DeltaL == DeltaR) {
    APCount();
  }
  if (absolute(DeltaL) != absolute(DeltaR) && DeltaL != DeltaR) {
    APOTrig();
  }
}

void AOPivot() {
  BotAngle = ITDW((DeltaL + DeltaR) / 2) / TurnRate;
  AOCalc(BotAngle);
}

void APCount() {
  Dist = (DeltaL + DeltaR) / 2;
  Ang = rad(CurrentO);
  Yoffset = Dist * sin(Ang);
  Xoffset = Dist * cos(Ang);
}

void APOTrig() {
  if (DeltaL > DeltaR) {
    float FMS = DeltaL;
    float SMS = DeltaR;
  }
  if (DeltaR > DeltaL) {
    float FMS = DeltaR;
    float SMS = DeltaL;
  }

  float TurnRadius = AxelRadius * (FMS / (FMS - SMS));
  float angb = 180 - absolute(CurrentO);
  float RadX = TurnRadius * sin(angb);
  float RadY = TurnRadius * cos(angb);

  float RotPointX = CurrentX + RadX;
  float RotPointY = CurrentY + RadY;

  float TranslationX = RotPointX;
  float TranslationY = RotPointY;
  float NewRotPointX = RotPointX - TranslationX;
  float NewRotPointY = RotPointY - TranslationY;

  float r = atan(RadY / RadX);
  float XnL = (xL * cos(r)) - (yL * sin(r));
  float YnL = (yL * cos(r)) + (xL * sin(r));

  float XnR = (xR * cos(r)) - (yR * sin(r));
  float YnR = (yR * cos(r)) + (xR * sin(r));

  float Rx = (XnL + XnR) / 2;
  float Ry = (YnL + YnR) / 2;

  float RnX = Rx - TranslationX;
  float RnY = Ry - TranslationY;

  CurrentO += r;
  CurrentX += RnX;
  CurrentY += RnY;
}

void AOCalc(float Ooffset) { CurrentO += Ooffset; }

void APCalc(float Xoffset, float Yoffset) {
  CurrentY += Yoffset;
  CurrentX += Xoffset;
}

/////////////////
//  HEPHAESTUS //
/////////////////

/////////////////
//   ARTEMIS   //
/////////////////

/////////////////
//   HERMES    //
/////////////////

/////////////////
// MISCELLANEOUS//
/////////////////

char Condition() { return 'a'; }

int MOE(float InputA, float InputB, float Range) { return 0; }

void Test() {}

float absolute(float value) {
  if (value < 0) {
    value = value * -1;
  }
  return value;
}

float rad(float degrees) {
  float rad = degrees * 0.017453;
  return rad;
}

void Velocity() {
  float VLo = gmpc(Leftm);
  float VRo = gmpc(Rightm);
  msleep(10);
  float VLt = gmpc(Leftm);
  float VRt = gmpc(Rightm);
  float Lv = VLt - VLo;
  float Rv = VRt - VRo;
}

/////////////////
//     ARM     //
/////////////////

void ArmVertical(float Position, float Time) {}

void ConstThread() {
  if (stopThread == 1) {
    return;
  }
  Condition();
  msleep(100);
}

////////////////////////////////////////////////////////
// MMMMMMMMMMMMMMMMMMMk.        .kMMMMMMMMMMMMMMMMMMM //
// MMMMMMMMWNNWMMMMMMWo          oWMMMMMMWNNWMMMMMMMM //
// MMMMMMWKo,'ckNMWX0d'          'd0XWMNkc',oKWMMMMMM //
// MMMMWKo.     ,c:..              ..:c'     .oKWMMMM //
// MMMNx.                                      .xNMMM //
// MMMNo.                ......                .oNMMM //
// MMMMWk,          .,ldO0KKKK0kdc,.          ,kWMMMM //
// MMMMMWd.       .l0NMMMMMMMMMMMMN0l.       .dWMMMMM //
// MMMMMO'      .cKWMMMMMMMMMMMMMMMMWKl.      'OMMMMM //
// K0Okd,      .dNMMMMMMMMMMMMMMMMMMMMNd.      ,dkO0K //
// .           cNMMMMMMMMMMMMMMMMMMMMMMNc           . //
//            .kMMMMMMMMMMMMMMMMMMMMMMMMk.            //
//            .OMMMMMMMMMMMMMMMMMMMMMMMMO.            //
//            .dWMMMMMMMMMMMMMMMMMMMMMMWd.            //
// l:;,'.      ,0MMMMMMMMMMMMMMMMMMMMMM0,      .';;:l //
// MMWWNo.      ,0WMMMMMMMMMMMMMMMMMMW0,      .oNWMMM //
// MMMMMXc       .oKWMMMMMMMMMMMMMMWKo.       cXMMMMM //
// MMMMMNo.        .cxKNWMMMMMMWNKxc.        .oNMMMMM //
// MMMW0:             .';cllllc;'.             :0WMMM //
// MMMNc                                        cNMMM //
// MMMMXo.       .                    .       .oKMMMM //
// MMMMMWKo.  .;x0Ooc,.          .,coO0x;.  .oKWMMMMM //
// MMMMMMMWKxd0WMMMMMXc          cNMMMMMW0dxKWMMMMMMM //
// MMMMMMMMMMMMMMMMMMWd.        .dWMMMMMMMMMMMMMMMMMM //
// MMMMMMMMMMMMMMMMMMMO'        'OMMMMMMMMMMMMMMMMMMM //
////////////////////////////////////////////////////////

int main() {
  Startup();
  while(Frontb == 0){
	  Wait(0.01);
  }
  TankSpeed(-50,50,5);
  while(Leftbb == 0){
	  Wait(0.01);
  }
  SlowSetArm(45,3);
  while(Rightb == 0){
	  Wait(0.01);
  }
  SetArm(45,0);
  return 0;
}