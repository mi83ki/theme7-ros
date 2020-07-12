/***********************************************************************/
/*                                                                     */
/*  FILE        :Arduino2.ino                                          */
/*  DATE        :May 10, 2020                                          */
/*  DESCRIPTION :ROSロボ Arduino2用プログラム                          */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/

#include <Wire.h>
#include "CTimer.hpp"
#include "CBuzzer.hpp"
#include "CEncoder.hpp"
#include "CComArduinos.hpp"

/***********************************************************************/
/*                           グローバル変数                            */
/***********************************************************************/
// エンコーダカウントクラス
static CEncoder ge;
// 外部割込みINT6
ISR(INT6_vect) {
  ge.interruptEnc0A();
}
// ピン変化割込みPCINT0
ISR(PCINT0_vect) {
  ge.interruptEnc1B();
}

// Arduino間通信クラス
static CComArduinos gc(MASTER);
static uint8_t updateFlag = 0;

// ブザー駆動クラス
static CBuzzer *gb;

/***********************************************************************/
/*                               main関数                              */
/***********************************************************************/

// タイマーの初期化
void initTimer1(void) {
  //------------------------
  //  タイマカウンタ1の設定
  //------------------------
  TCCR1A = (0 << COM1A1)|(0 << COM1B1); // 標準ポート動作
  TCCR1A |= (1 << WGM11);               // 高速PWM動作（TOP値：ICR1）
  TCCR1B = (1 << WGM13)|(1 << WGM12);   // 高速PWM動作（TOP値：ICR1）
  ICR1 = 2500 - 1;                      // 16MHzクロック64分周で100HzのPWM波形＆割込み生成
  TIMSK1 = (1 << ICIE1);                // ﾀｲﾏ/ｶｳﾝﾀ1捕獲割り込み許可
  sei();                                // 全割込み許可
  TCCR1B |= (1 << CS11)|(1 << CS10);    // 64分周でタイマースタート
}

/* 10ms毎の割り込み */
ISR(TIMER1_CAPT_vect) {
  // 現在値を更新
  gc.A2state.time = CTimer::getGlobalTime();
  gc.A2state.encR = ge.getEncR();
  gc.A2state.encL = ge.getEncL();
  updateFlag++;
}


/***********************************************************************/
/*                               main関数                              */
/***********************************************************************/
void setup() {
  // Timer1の設定
  initTimer1();

  // ブザーの設定
  static CBuzzer sb;
  gb = &sb;
  gb->startMelody(shotMelody1);
  while(gb->driveBuzzer());

  // シリアルデバッグ用
  Serial.begin(115200);
  delay(1000);
  Serial.println("Arduino2");
}

void loop() {
  if (updateFlag) {
    updateFlag = 0;
    // Arduino1へ送信
    gc.i2cMasterTransmit();
    //Serial.print(gc.A2state.encR);
    //Serial.print(", ");
    //Serial.print(gc.A2state.encL);
    //Serial.print(", ");
    //Serial.println(gc.A2state.time);
  }
}
