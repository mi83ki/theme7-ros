//#include <iostream.h>
#pragma once

#include "CComArduinos.hpp"
#include "pid.hpp"
#include "fix.hpp"


typedef struct arduino1_state{
  fix omega_right;
  fix omega_left;
  fix vel_right;
  fix vel_left;
} arduino1StateType;


class SpeedControll
{
  public:
    SpeedControll();
    ~SpeedControll();

    /***********************************************************************/
    /*                           速度制御関数                              */
    /***********************************************************************/
    void calcSpeed(arduino2StateType A2state, arduino1StateType *A1st);
    void controllMotorsSpeed(fix torque_pid_right, fix torque_pid_left, arduino1StateType A1st);

  public:
    typedef struct pidDuty {
      int32_t duty_right;
      int32_t duty_left;
    } pidDutyType;

    pidDutyType duty_status;

  private:
    const float RESISTANCE = 10.0f;          // 抵抗値[]
    const float MAX_VOLT = 5.0f;             // 電圧値[V]
    const float KR_TORQUE = 0.1523f;        // トルク定数[]
    const float KE_VOLT = 0.1523f;          // 逆起電力定数[V]
    const int GEAR_RATIO = 150;         // ギア比
    const int ENCODER_PULSE = 3;        // パルス数
    const float TIRE_DIAMETER = 0.09f;     // タイヤの直径[m]
    const int DUTY_RESOLUTION_ = 100;
};
