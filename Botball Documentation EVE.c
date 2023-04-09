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
int Sorter = 0;
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
//    other    //
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
float RMM = 1;
////////////////////

////////////////////
//  DECLARATIONS  //
////////////////////

float absolute(float);

int MOE(float, float, float);

void Test();

void ArmVertical(float, float);

float rad(float);

void TankSpeed(float, float, float);

void TankRotation(float, float, float);

void TankDistance(float, float, float);

void DriveSpeed(float, float, float);

void DriveRotation(float, float, float);

void DriveDistance(float, float, float);

void SetArm(float, float);

void ChangeArm(float, float);

void SlowSetArm(float, float);

int DTTW(float);

float TTDW(int);

float DTIW(float);

float ITDW(float);

float TTIW(int);

int ITTW(float);

int DTTA(float);

float TTDA(int);

void Wait(float);

void Brake();

void CMR();

void HandsOff();

void Startup();

////////////////////
///////////////////
//   MOVEMENT  ///
/////////////////

void TankSpeed(float SpeedL, float SpeedR, float Time) {
  Lv = SpeedL;
  Rv = SpeedR;

  create_drive_direct(-(SpeedL * 6) * LMM, -(SpeedR * 6) * RMM);
  msleep(Time * 1000);
}

void TankRotation(float RotL, float RotR, float Time) {
  int RotLn = DTTW(RotL);
  int RotRn = DTTW(RotR);
  float SpeedL = RotLn / Time;
  float SpeedR = RotRn / Time;

  TankSpeed(SpeedL, SpeedR, Time);
}

void TankDistance(float DistL, float DistR, float Time) {
  int DistLn = DTIW(DistL);
  int DistRn = DTIW(DistR);
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
  set_servo_enabled(Sorter, DTTA(PosA));
  set_servo_enabled(Arm, DTTA(PosC));
}

void ChangeArm(float DeltaA, float DeltaC) {
  int CurA = TTDA(get_servo_position(Sorter));
  int CurC = TTDA(get_servo_position(Arm));
  int PosA = CurA + DeltaA;
  int PosC = CurC + DeltaC;

  SetArm(PosA, PosC);
}

void SlowSetArm(float Pos, float Time) {
  int CurPos = get_servo_position(Sorter);
  int distance = Pos - CurPos;
  int Distn = distance / 50;
  int Timen = Time / 50;
  int count = 0;
  while (count < 50) {
    ChangeArm(Distn, 0);
    msleep(Timen);
    count += 1;
  }
}

/////////////////
// CONVERSIONS //
//    wheels   //
/////////////////

int DTTW(float Degrees) {
  int Ticks = Degrees / 5.55;
  return Ticks;
}

float TTDW(int Ticks) {
  float Degrees = Ticks * 5.55;
  return Degrees;
}

float DTIW(float Degrees) {
  float Inches = Degrees * 3.14;
  return Inches;
}

float ITDW(float Inches) {
  float Degrees = Inches / 3.14;
  return Degrees;
}

float TTIW(int Ticks) {
  float Inches = Ticks * 134.431;
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
  int Tics = (0.09 / Degrees) + 57.15;
  return Tics;
}

float TTDA(int Tics) {
  float Degrees = (0.09 * Tics) - 57.15;
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
  printf("and god said... /n");
  wait_for_light(Starts);
  shut_down_in(119);
  printf("...let there be light /n");
}

void Startup() {
  create_connect();
  Brake();
  CMR();
  SetArm(0, 0);
  console_clear();
}

int MOE(float InputA, float InputB, float Range) { return 0; }

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
  Startup();
  HandsOff();
  TankSpeed(50, 100, 10);
  return 0;
}