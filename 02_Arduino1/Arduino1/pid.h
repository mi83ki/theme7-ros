/***********************************************************************/
/*                                                                     */
/*  FILE        :pid.h                                                 */
/*  DATE        :2017/3/18                                             */
/*  DESCRIPTION :PID制御に関する関数を揃えたヘッダ                     */
/*  CPU TYPE    :EV3                                                   */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki.                        */
/*                                                                     */
/***********************************************************************/

/*************************  pid.c の使い方  ****************************/
/*                                                                     */
/* ①PID制御部1つにつき、1つのPID制御構造体を定義する。                */
/* ②その構造体を initPID() で初期化する。                             */
/* ③FILT_FREQ の周波数で pidControl() し、その戻り値が制御量である。  */
/*                                                                     */
/*---------------------------------------------------------------------*/
/*      例 (while内のif文がFILT_FREQで周期的に実行されるようにする)    */
/*---------------------------------------------------------------------*/
/* #include "pid.c"                                                    */
/*                                                                     */
/* int main(void) {                                                    */
/*   pidType pid1;   // PID制御構造体を作成                            */
/*   fix v;          // 現在速度                                       */
/*   fix power;      // モーターデューティ比                           */
/*                                                                     */
/*   // 初期設定                                                       */
/*   // pid1の現在値をvに、目標値を10.0[cm/s]に、比例ゲインを1.0に、   */
/*   // 積分時間を0.6に、微分時間を0.2に設定                           */
/*   initPID(&pid1, KTT, &v, INT_TO_FIX(10), 1.0, 0.6, 0.2);           */
/*                                                                     */
/*   while(1) {                                                        */
/*     if (FILT_FREQごとに真) {                                        */
/*       v = getVelocity();    // 現在速度[cm/s]を取得（例）           */
/*                                                                     */
/*       // 制御量を取得                                               */
/*       power = pidControl(&pid1, FILT_FREQ);                         */
/*       // オーバーフロー対策 powerを-100～100に切り落とす            */
/*       fixcutoff(&power, INT_TO_FIX(100), -INT_TO_FIX(100))          */
/*                                                                     */
/*       driveMotor(FIX_TO_INT(power));    // モーターを駆動（例）     */
/*     }                                                               */
/*   }                                                                 */
/*   return(0);                                                        */
/* }                                                                   */
/*                                                                     */
/***********************************************************************/
#ifndef _PID_H__
#define _PID_H__

#include "fix.h"

#define MAX_OF_S32BIT  2147483647  /* 符号あり32ビットの最大値 */
#define MAX_OF_S64BIT 9223372036854775807 /* 符号あり64ビットの最大値 */


/***********************************************************************/
/*                          関数形式マクロ                             */
/***********************************************************************/

/* 値を積分する(引数: 現在値，前回積分値，サンプリング周波数) */
#ifndef INTEGRATE
#define INTEGRATE(pre,last,f) ((last) + ((pre) / (f)))
#endif
/* 台形則で積分する(引数: 現在値，前回積分値，サンプリング周波数) */
/* 使用例：　x += TRAP_INTEGRATE(v, former_v, SAMPLE_FREQ) */
#ifndef TRAP_INTEGRATE
#define TRAP_INTEGRATE(pre,last,f) (((pre) + (last)) / 2 / (f))
#endif
/* 値を微分する(引数: 現在値，前回値，サンプリング周波数) */
#ifndef DIFFERENTIATE
#define DIFFERENTIATE(pre,last,f) (((pre) - (last)) * (f))
#endif

/***********************************************************************/


typedef struct pid {        /* PID制御に関する構造体 */
  uint8_t en;             /* 有効かどうか */
  enum ePID {KKK, KTT} mode;    /* Kp,Ki,KdかKp,Ti,Tdどっちか */
  struct {
    fix p;                          /* 比例ゲイン */
    fix i;                          /* 積分ゲイン or 積分時間 */
    fix d;                          /* 微分ゲイン or 微分時間 */
  } gain;                           /* ゲイン */
  struct {
    fix lastError;          /* 一つ前の偏差（微分用） */
    fix integError;         /* 偏差の積分値（積分用） */
  } buf;
  fix *present;           /* 現在値 */
  fix desired;            /* 目標値 */
} pidType;


#endif

