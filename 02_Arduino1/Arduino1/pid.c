/***********************************************************************/
/*                                                                     */
/*  FILE        :pid.c                                                 */
/*  DATE        :2017/3/18                                             */
/*  DESCRIPTION :PID制御に関する関数を揃えたソース                     */
/*  CPU TYPE    :EV3                                                   */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki.                        */
/*                                                                     */
/***********************************************************************/

#ifndef _PID_C__
#define _PID_C__

#include "pid.h"

#ifndef _MYSTDLIB_C__

/* max以上はmaxに，min以下はminに切り落とす */
/* オーバーフローで 1 を，アンダーフローで -1 を返す */
char icutoff(int *val, int max, int min) {
  if (*val < min) {
    *val = min;
    return(-1);
  } else if (*val > max) {
    *val = max;
    return(1);
  } else {
    return(0);
  }
}

char fixcutoff(fix *val, fix max, fix min) {
  if (*val < min) {
    *val = min;
    return(-1);
  } else if (*val > max) {
    *val = max;
    return(1);
  } else {
    return(0);
  }
}

char llcutoff(int64_t *val, int64_t max, int64_t min) {
  if (*val < min) {
    *val = min;
    return(-1);
  } else if (*val > max) {
    *val = max;
    return(1);
  } else {
    return(0);
  }
}
#endif

/* PID制御構造体の初期化関数 */
void initPID(pidType *pid, uint8_t md, fix *pre, fix des,
                    float p, float i, float d) {
  pid->en = 0;
  pid->mode = md;
  pid->gain.p = FLOAT_TO_FIX(p);
  pid->gain.i = FLOAT_TO_FIX(i);
  pid->gain.d = FLOAT_TO_FIX(d);
  pid->present = pre;
  pid->desired = des;
  pid->buf.lastError = 0;
  pid->buf.integError = 0;
}

/* PID制御関数 */
fix pidControl(pidType *pid, int16_t freq) {
  int64_t output = 0;
  fix error = pid->desired - *(pid->present);    /* 偏差 */

  switch (pid->mode) {
  case KKK:
    /* 比例動作 */
    output = FIX_MUL(pid->gain.p, error);

    /* 積分動作 */
    pid->buf.integError += TRAP_INTEGRATE(error, pid->buf.lastError, freq);
    output += FIX_MUL(pid->gain.i, pid->buf.integError);
   
    /* 微分動作 */
    output += FIX_MUL(pid->gain.d, DIFFERENTIATE(error, pid->buf.lastError, freq));
    pid->buf.lastError = error;
    break;
  case KTT:
    /* 比例動作 */
    output = (int64_t)error;

    /* 積分動作 */
    pid->buf.integError += TRAP_INTEGRATE(error, pid->buf.lastError, freq);
    output += FIX_DIV(pid->buf.integError, pid->gain.i);

    /* 微分動作 */
    output += FIX_MUL(pid->gain.d, DIFFERENTIATE(error, pid->buf.lastError, freq));
    pid->buf.lastError = error;

    output = FIX_MUL(pid->gain.p, output);
    break;
  default:
    break;
  }
  /* int64_t から fix に戻すときのオーバーフローを回避するため */
  llcutoff(&output, MAX_OF_S32BIT, -MAX_OF_S32BIT);

  return ((fix)output);
}

/* 積分器をサチらせる */
int8_t integErrCutoff(pidType *pid, fix max, fix min) {
  int64_t integErrMax = 0, integErrMin = 0;
  switch (pid->mode) {
  case KKK:
    /* max / Ki */
    integErrMax = FIX_DIV(max, pid->gain.i);
    /* min / Ki */
    integErrMin = FIX_DIV(min, pid->gain.i);
    break;
  case KTT:
    /* Ti × max / Kp */
    integErrMax = FIX_MUL(pid->gain.i, max);
    integErrMax = FIX_DIV(integErrMax, pid->gain.p);
    /* Ti × min / Kp */
    integErrMin = FIX_MUL(pid->gain.i, min);
    integErrMin = FIX_DIV(integErrMin, pid->gain.p);
    break;
  default:
    break;
  }
  return(fixcutoff(&(pid->buf.integError), integErrMax, integErrMin));
}


/* 偏差の積分値を切り落とす */
//fix intErrFilter(pidType *pid, int64_t out) {
//  if (pid->gain[1].nume) {
//    out *= (int64_t)pid->gain[1].deno;
//    out /= (int64_t)pid->gain[1].nume;
//    pid->buf.integError = (fix)out;
//    return (pid->buf.integError);
//  } else {
//    return(0);
//  }
//}

/* pidTypeと目標値をもらって，PID制御をスタートするときの関数 */
void startPID(pidType *pid, fix des) {
  pid->en = 1;  /* 有効にして */
  pid->buf.lastError = 0;
  pid->buf.integError = 0;  /* バッファクリア */
  pid->desired = des;
}

/* PID制御を止めるときの関数 */
void stopPID(pidType *pid) {
  pid->en = 0;  /* 無効にする */
}

#endif

