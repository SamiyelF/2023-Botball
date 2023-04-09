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
//    other	   //
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
int stopThread = 0;
////////////////////
//ignore
float timemult = 1;
float WMM = 1;
////////////////////
//  DECLARATIONS  //
////////////////////

float absolute(float);
// in (value)
// out (absolute value)
// Get the absolute value of an input
// abs() wasnt working for whatever reason idk

float rad(float);
// in (degrees)
// converts degrees to radians
// math.h's was busted idk
// out (radians)

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

////////////////////
///////////////////
//   MOVEMENT  ///
/////////////////

void TankSpeed(float SpeedL, float SpeedR, float Time) {
  motor(Leftm, SpeedL * LMM);
  motor(Rightm, SpeedR * RMM);
  msleep((Time * 1000) * timemult);
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
//MISCELLANEOUS//
/////////////////

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
  printf("hello \n");
  Startup();
  HandsOff();
  TankSpeed(50,50,10);
  return 0;
}
