#include "SpeedController.hpp"

SpeedControll::SpeedControll()
{
  ;
}

SpeedControll::~SpeedControll()
{
  ;
}

//arduino2のエンコーダ値と時間からタイヤの速度[m/s][rad/s]を更新する
void SpeedControll::calcSpeed(arduino2StateType A2state, arduino1StateType *A1st) {
  static int32_t last_encR = 0;
  static int32_t last_encL = 0;
  static uint32_t last_time = 0;       // ms
  static const fix myPI = FLOAT_TO_FIX(M_PI);
  static const fix myTireDiameter = FLOAT_TO_FIX(TIRE_DIAMETER);
  static const fix myGearRagtio = INT_TO_FIX(GEAR_RATIO);
  static const fix myEncoderPulse = INT_TO_FIX(ENCODER_PULSE);
  static const fix temp = FIX_MUL(myGearRagtio, myEncoderPulse); //ok
  static const fix temp2 = FIX_MUL(myTireDiameter, myPI);        //ok
  static fix diff_time = 0;       // m/s

  diff_time = INT_TO_FIX(A2state.time - last_time) / 1000;

//  Serial.print("diff_time:");
//  Serial.print(FIX_TO_FLOAT(diff_time));
//  Serial.print(',');

  if (diff_time != 0) {
    A1st->omega_right = 2 * FIX_MUL(INT_TO_FIX(A2state.encR - last_encR), myPI);
    A1st->omega_right = FIX_DIV(A1st->omega_right, FIX_MUL(temp, diff_time));
    A1st->omega_left = 2 * FIX_MUL(INT_TO_FIX(A2state.encL - last_encL), myPI);
    A1st->omega_left = FIX_DIV(A1st->omega_left, FIX_MUL(temp, diff_time));

    A1st->vel_right = FIX_MUL(A1st->omega_right, (myTireDiameter / 2));
    A1st->vel_left = FIX_MUL(A1st->omega_left, (myTireDiameter / 2));

    Serial.print("present:");
    Serial.print(FIX_TO_FLOAT(A1st->vel_right));
    Serial.print(',');
    Serial.print(FIX_TO_FLOAT(A1st->vel_left));
    Serial.println(',');
  }

  last_encR = A2state.encR;
  last_encL = A2state.encL;
  last_time = A2state.time;
}

//受け取った目標速度にするためにトルクを計算してモーターを制御する
void SpeedControll::controllMotorsSpeed(fix torque_pid_right, fix torque_pid_left, arduino1StateType A1st) {
  fix duty_right = 0;
  fix duty_left = 0;
  static const fix myResistance = FLOAT_TO_FIX(RESISTANCE);
  static const fix myKrTorque = FLOAT_TO_FIX(KR_TORQUE);
  static const fix myKeVolt = FLOAT_TO_FIX(KE_VOLT);
  static const fix myMaxVolt = FLOAT_TO_FIX(MAX_VOLT);
  static const fix myDutyResolution = INT_TO_FIX(DUTY_RESOLUTION_);
  static const fix temp = FIX_DIV(myDutyResolution, myMaxVolt);

  duty_right = (FIX_MUL(myResistance, torque_pid_right) / KR_TORQUE) + FIX_MUL(myKeVolt, A1st.omega_right);
//  Serial.print("duty1:");
//  Serial.print(FIX_TO_INT(duty_right));
//  Serial.print(',');
  duty_right = FIX_MUL(duty_right, temp);
  duty_left = (FIX_MUL(myResistance, torque_pid_left) / KR_TORQUE) + FIX_MUL(myKeVolt, A1st.omega_left);
//  Serial.print(FIX_TO_INT(duty_left));
//  Serial.print(',');
  duty_left = FIX_MUL(duty_left, temp);

  fixcutoff(&duty_right, INT_TO_FIX(100), -INT_TO_FIX(100));
  fixcutoff(&duty_left, INT_TO_FIX(100), -INT_TO_FIX(100));
    
  Serial.print("duty:");
  Serial.print(FIX_TO_INT(duty_right));
  Serial.print(',');
  Serial.print(FIX_TO_INT(duty_left));
  Serial.print(',');

  duty_status.duty_right = FIX_TO_INT(duty_right);
  duty_status.duty_left = FIX_TO_INT(duty_left);
  //driveMotors(FIX_TO_INT(duty_right), FIX_TO_INT(duty_left));
}
