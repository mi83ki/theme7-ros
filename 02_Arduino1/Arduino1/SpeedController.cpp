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
  static int32_t last_encR = 0;        //前回のエンコーダ値
  static int32_t last_encL = 0;        //前回のエンコーダ値
  static int32_t sum_encR = 0;         //100ms のエンコーダ値の積算値
  static int32_t sum_encL = 0;         //100ms のエンコーダ値の積算値
  static uint32_t last_time = 0;       // ms
  static uint32_t sum_time = 0;       // ms
  static const fix myPI = FLOAT_TO_FIX(M_PI);
  static const fix myTireDiameter = FLOAT_TO_FIX(TIRE_DIAMETER);
  static const fix myGearRagtio = INT_TO_FIX(GEAR_RATIO);
  static const fix myEncoderPulse = INT_TO_FIX(ENCODER_PULSE);
  static const fix temp = FIX_MUL(myGearRagtio, myEncoderPulse);
  static fix diff_time = 0;       // m/s

  diff_time = INT_TO_FIX(A2state.time - last_time) / 1000;

  //前回との差分の積算
  sum_time += diff_time;
  sum_encR += A2state.encR - last_encR;
  sum_encL += A2state.encL - last_encL;

  if (diff_time != 0) {
    if (sum_time >= 100)
    {
      //角速度の計算
      A1st->omega_right = 2 * FIX_MUL(INT_TO_FIX(sum_encR), myPI);
      A1st->omega_right = FIX_DIV(A1st->omega_right, FIX_MUL(temp, sum_time));
      A1st->omega_left = 2 * FIX_MUL(INT_TO_FIX(sum_encL), myPI);
      A1st->omega_left = FIX_DIV(A1st->omega_left, FIX_MUL(temp, sum_time));

      //速度の計算
      A1st->vel_right = FIX_MUL(A1st->omega_right, (myTireDiameter / 2));
      A1st->vel_left = FIX_MUL(A1st->omega_left, (myTireDiameter / 2));

      //カウンターのリセット
      sum_time = 0;
      sum_encR = 0;
      sum_encL = 0;
    }
  }

  //現在の値を保持
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

  //デューティ比計算
  duty_right = (FIX_MUL(myResistance, torque_pid_right) / KR_TORQUE) + FIX_MUL(myKeVolt, A1st.omega_right);
  duty_left = (FIX_MUL(myResistance, torque_pid_left) / KR_TORQUE) + FIX_MUL(myKeVolt, A1st.omega_left);
  duty_right = FIX_MUL(duty_right, temp);
  duty_left = FIX_MUL(duty_left, temp);

  //デューティ比の最大・最小値制限
  fixcutoff(&duty_right, INT_TO_FIX(100), -INT_TO_FIX(100));
  fixcutoff(&duty_left, INT_TO_FIX(100), -INT_TO_FIX(100));

  //デューティ比をint型に変換
  duty_status.duty_right = FIX_TO_INT(duty_right);
  duty_status.duty_left = FIX_TO_INT(duty_left);
  //driveMotors(FIX_TO_INT(duty_right), FIX_TO_INT(duty_left));
}
