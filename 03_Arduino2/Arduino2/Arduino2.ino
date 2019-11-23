/***********************************************************************/
/*                                                                     */
/*  FILE        :arduino2-r05.ino                                      */
/*  DATE        :Nov 19, 2019                                          */
/*  DESCRIPTION :ROSロボ Arduino2用プログラム                          */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/*  メモ：                                                             */
/*        R00：encode2のテスト                                         */
/*        R01：decode2のテスト                                         */
/*        R02：Arduino1との通信テスト                                  */
/*        R03：Arduino1とI2Cで通信テスト                               */
/*        R04：Arduino2の通信メッセージにtimeを追加                    */
/*        R05：エンコーダ値のトピックを立てる                          */
/*                                                                     */
/***********************************************************************/

#include <Wire.h>
#include "comA1andA2.hpp"


/***********************************************************************/
/*                             CountEncoder                            */
/*---------------------------------------------------------------------*/
/*                       エンコーダをカウントする                      */
/***********************************************************************/
// エンコーダのピン
#define ENC0A 2
#define ENC0B 8
#define ENC1A 3
#define ENC1B 14

// エンコーダのカウント値
static int32_t encR, encL;

// 初期設定
void initCountEncoder(void) {
  pinMode(ENC0A, INPUT_PULLUP);  // エンコーダ0のAチャンネルを入力に設定
  pinMode(ENC0B, INPUT_PULLUP);  // エンコーダ0のBチャンネルを入力に設定
  pinMode(ENC1A, INPUT_PULLUP);  // エンコーダ1のAチャンネルを入力に設定
  pinMode(ENC1B, INPUT_PULLUP);  // エンコーダ1のBチャンネルを入力に設定
  attachInterrupt(0, interruptEnc0A, FALLING);   // INT0の外部割込みを立下りで設定
  attachInterrupt(1, interruptEnc1A, FALLING);   // INT1の外部割込みを立下りで設定
}

// エンコーダーRの割込み
void interruptEnc0A (void) {
  if (digitalRead(ENC0B) == 0) {
    encR++;
  } else {
    encR--;
  }
}

// エンコーダーLの割込み
void interruptEnc1A (void) {
  if (digitalRead(ENC1B) == 0) {
    //encL++;
    encL--;
  } else {
    //encL--;
    encL++;
  }
}

void getEncoderCounts(int32_t *eR, int32_t *eL) {
  *eR = encR;
  *eL = encL;
}

void writeEncoderCounts(int32_t eR, int32_t eL) {
  encR = eR;
  encL = eL;
}

/***********************************************************************/
/*                           タイマー関数                              */
/*        注：initMotor()によって1msの割込みが設定されている           */
/***********************************************************************/
// タイマーの初期化
void initTimer1(void) {
  //------------------------
  //  タイマカウンタ1の設定
  //------------------------
  //TCCR1A = (1 << COM1A1)|(1 << COM1B1); // 比較一致でLow、BOTTOMでHighをOC1xﾋﾟﾝへ出力 (非反転動作)
  TCCR1A = (0 << COM1A1)|(0 << COM1B1); // 標準ポート動作
  TCCR1A |= (1 << WGM11);               // 高速PWM動作（TOP値：ICR1）
  TCCR1B = (1 << WGM13)|(1 << WGM12);   // 高速PWM動作（TOP値：ICR1）
  //OCR1A = PWM_RESOLUTION;               // 初期値
  //OCR1B = PWM_RESOLUTION;               // 初期値
  //ICR1 = 7999;                         // 8MHzクロック前置分周なしで1kHzのPWM波形＆割込み生成
  ICR1 = 1249;                          // 8MHzクロック64分周で100HzのPWM波形＆割込み生成
  //ICR1 = 2499;                          // 8MHzクロック64分周で50HzのPWM波形＆割込み生成
  //ICR1 = 12499;                          // 8MHzクロック64分周で10HzのPWM波形＆割込み生成
  TIMSK1 = (1 << ICIE1);                // ﾀｲﾏ/ｶｳﾝﾀ1捕獲割り込み許可
  sei();                                // 全割込み許可
  //TCCR1B |= (1 << CS10);                // 前置分周なしでタイマースタート
  //TCCR1B |= (1 << CS11);                // 8分周でタイマースタート
  TCCR1B |= (1 << CS11)|(1 << CS10);    // 64分周でタイマースタート
}

uint32_t tempTimer;  // 1msの割り込みでインクリメントされる変数

// タイマーをクリアする
void startTimer(void) {
  tempTimer = millis();
}

// タイマー値[ms]を取得する
uint32_t getTime(void) {
  return(millis() - tempTimer);
}

// タイマー値[ms]を取得する
uint32_t getGlobalTime(void) {
  return(millis());
}



/***********************************************************************/
/*                               main関数                              */
/***********************************************************************/
// Arduino2の状態量
static arduino2StateType A2state;
static uint8_t updateFlag = 0;

void samplingA2(arduino2StateType *a2st) {
  A2state.time = getGlobalTime();
  getEncoderCounts(&A2state.encR, &A2state.encL);
}

/* 10ms毎の割り込み */
ISR(TIMER1_CAPT_vect) {
  // 現在値を更新
  samplingA2(&A2state);
  updateFlag++;
}


void setup() {
  // シリアルデバッグ用
  Serial.begin(115200);
  Serial.println("Arduino2");

  // エンコーダ
  initCountEncoder();

  // Arduiono1との通信設定
  initComA1andA2(MASTER, &A2state);

  // Timer1の設定
  initTimer1();
}

void loop() {
  if (updateFlag) {
    updateFlag = 0;
    // Arduino1へ送信
    i2cMasterTransmit(A2state);
    //Serial.print(A2state.encR);
    //Serial.print(", ");
    //Serial.print(A2state.encL);
    //Serial.print(", ");
    //Serial.println(A2state.time);
  }
}

