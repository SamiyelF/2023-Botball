#include <stdio.h>
#include <kipr/wombat.h>
#include <math.h>

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
	float WheelR = 2;
	float AxelRadius = 4;
	float TurnRate = 2; //AxelRadius / WheelR
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
////////////////////


////////////////////
//  DECLARATIONS  //
////////////////////
	
	float absolute(float);
	
	void CollisionCheck();
	
	void CollisionReset();
	
	void CollisionRecovery(float, float, float);
	
	void AthenaDecision(float, float);
	
	void AOPivot();
	
	void APCount();
	
	void APOTrig();
	
	void AOCalc(float);
	
	void APCalc(float, float);
	
	char condition();
	
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

	void TankAccelerate(float, float, int);

	void DriveAccelerate(float, float, int);

	void SmoothTankTime(float, float, float, float, int, float);

	void SmoothDriveTime(float, float, float, float, int, float);

	void SetArm(float, float);

	void ChangeArm(float, float);

	void SlowSetArm(float, float);

	int DTTW (float);

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

	void Velocity();

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
	/////////////////

	void TankAccelerate(float GoalSpeedL, float GoalSpeedR, int Acceleration) {
		float CurrentSpeedL = Lv;
		float CurrentSpeedR = Rv;
		float SpeedDifL = GoalSpeedL - CurrentSpeedL;
		float SpeedDifR = GoalSpeedR - CurrentSpeedR;
		SpeedDifL = SpeedDifL / 25;
		SpeedDifR = SpeedDifR / 25;
		float SpeedL = CurrentSpeedL;
		float SpeedR = CurrentSpeedR;
		int count = 0;
		while (count < 25){
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
		while (count < 25){
			DriveSpeed (SpeedB, SpeedD, (Acceleration / 1000));
			SpeedB += SpeedDifB;
			SpeedD += SpeedDifD;
		}
	}

	/////////////////

	void SmoothTankTime(float TopL, float TopR, float BotL, float BotR, int Acceleration, float Time) {
		TankAccelerate(TopL, TopR, Acceleration);
		TankSpeed(TopL, TopR, Time);
		TankAccelerate(BotL, BotR, Acceleration);
	}

	void SmoothDriveTime(float TopB, float TopD, float BotB, float BotD, int Acceleration, float Time) {
		DriveAccelerate(TopB, TopD, Acceleration);
		DriveSpeed(TopB, TopD, Time);
		DriveAccelerate(BotB, BotD, Acceleration);
	}


/////////////////
//     ARM     // 
/////////////////

	void SetArm (float PosA, float PosC) {
		set_servo_enabled(Sorter, DTTA(PosA));
		set_servo_enabled(Arm, DTTA(PosC));
	}

	void ChangeArm (float DeltaA, float DeltaC) {
		int CurA = TTDA(get_servo_position(Sorter));
		int CurC = TTDA(get_servo_position(Arm));
		int PosA = CurA + DeltaA;
		int PosC = CurC + DeltaC;

		SetArm(PosA, PosC);
	}

	void SlowSetArm (float Pos, float Time) {
		int CurPos = get_servo_position(Sorter);
		int distance = Pos - CurPos;
		int Distn = distance / 50;
		int Timen = Time / 50;
		int count = 0;
		while (count < 50) {
		ChangeArm(Distn,0);
		msleep(Timen);
		count += 1;
		}
	}

/////////////////
// CONVERSIONS // 
//    wheels   //
/////////////////

	int DTTW (float Degrees) {
		int Ticks = Degrees / 5.55;
		return Ticks;
	}

	float TTDW (int Ticks) {
		float Degrees = Ticks * 5.55;
		return Degrees;
	}

	float DTIW (float Degrees) {
		float Inches = Degrees * 3.14;
		return Inches;
	}

	float ITDW (float Inches) {
		float Degrees = Inches / 3.14;
		return Degrees;
	}

	float TTIW (int Ticks) {
		float Inches = Ticks * 134.431;
		return Inches;
	}

	int ITTW (float Inches) {
		int Ticks = Inches / 314.413;
		return Ticks;
	}

/////////////////
// CONVERSIONS // 
//     arms    //
/////////////////

	int DTTA (float Degrees) {
		int Tics = (0.09 / Degrees) + 57.15;
		return Tics;
	}

	float TTDA (int Tics) {
		float Degrees = (0.09 * Tics) - 57.15;
		return Degrees;
	}

/////////////////
//MISCELLANEOUS// 
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

	void Brake(){
		freeze(Leftm);
		freeze(Rightm);
	}

	void CMR () {
		cmpc(Leftm);
		cmpc(Rightm);
	}

	void HandsOff () {
		printf("and god said... /n");
		wait_for_light(Starts);
		shut_down_in(119);
		printf("...let there be light /n");
	}

	void Startup () {
		create_connect();
		Brake();
		CMR();
		SetArm(0,0);
		console_clear();
	}

	void Velocity () {
		while(1) {
			int Lo = gmpc(Leftm);
			int Ro = gmpc(Rightm);
			msleep(10);
			int Lt = gmpc(Leftm);
			int Rt = gmpc(Rightm);

			Lv = Lt - Ro;
			Rv = Rt - Ro;
		}

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
		}
		else {
			Acceleration = 1;
		}
	
		if (MOE (DeltaL, 0, 10) == 1){
			Movement = 0;
		}
		else {
			Movement = 1;
		}
	
		if (Acceleration == 1 && Movement == 1) {
			Collision = 1;
		}
		else {
			Collision = 0;
		}
	}
	
	void CollisionReset() {
		//face bottom
		//CMR()
		//drive 
		//back up
		//Ydist = MPC
		//face left
		//CMR()
		//drive
		//back up
		//Xdist = MPC
		//return Xdist, Ydist
	}
	
	void CollisionRecovery(float x, float y, float o) {
		//if collided
		//run reset
		//adjust values for offset
		//trig n follow values
		//turn to orrientation
	}
	
	
/////////////////
// PATHFINDING // 
/////////////////


/////////////////
//   ATHENA    // 
/////////////////
	
	void AthenaDecision(float DeltaL, float DeltaR){
		if (absolute(DeltaL) == absolute(DeltaR) && DeltaL != DeltaR){
			AOPivot();
		}
		if (DeltaL == DeltaR){
			APCount();
		}
		if(absolute(DeltaL) != absolute(DeltaR) && DeltaL != DeltaR){
			APOTrig();
		}
	}
	
	void AOPivot () {
		BotAngle = ITDW((DeltaL + DeltaR) / 2) / TurnRate;
		AOCalc(BotAngle);	
	}
	
	void APCount () {
		Dist = (DeltaL + DeltaR) / 2;
		Ang = rad(CurrentO);
		Yoffset = Dist * sin(Ang);
		Xoffset = Dist * cos(Ang);
		
	}
	
	void APOTrig () {
		if (DeltaL > DeltaR){
			float FMS = DeltaL;
			float SMS = DeltaR;
		}
		if (DeltaR > DeltaL){
			float FMS = DeltaR;
			float SMS = DeltaL;
		}
			
		float TurnRadius = AxelRadius * (FMS/(FMS - SMS))	;
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
		float YnR = (yR * cos(r)) + (xR * sin(r)) ;
	
		float Rx = (XnL + XnR) / 2;
		float Ry = (YnL + YnR) / 2 ;
	
		float RnX = Rx - TranslationX;
		float RnY = Ry - TranslationY;
		
		CurrentO += r;
		CurrentX += RnX;
		CurrentY += RnY;
	}
	
	
	void AOCalc (float Ooffset) { 
		CurrentO += Ooffset;
	}
	
	void APCalc (float Xoffset, float Yoffset) {
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
//MISCELLANEOUS// 
/////////////////
	
	char Condition () {
		return 'a';
	}
	
	int MOE (float InputA, float InputB, float Range) {
		return 0;
	}
	
	void Test () {
	
	}
	
	float absolute(float value){
		if(value < 0) {
			value = value * -1;
		}
		return value;
	}
	
	float rad(float degrees){
		float rad = degrees * 0.017453;
		return rad;
	}
	
	
/////////////////
//     ARM     // 
/////////////////
	
	void ArmVertical (float Position, float Time) {
	
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
	set_servo_position(Arm,1400);
  enable_servos();
  while(get_create_lbump() == 0){
	  Wait(0.01);
  }
  TankSpeed(-50,50,5);
	set_servo_position(Arm, 0);
  while(get_create_lbump() == 0){
	  Wait(0.01);
  }
  set_servo_position(Sorter, 510);
  while(get_create_lbump() == 0){
	  Wait(0.01);
  }
  set_servo_position(Sorter,1560);
  return 0;
}