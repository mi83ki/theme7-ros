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
#include "comA1andA2.hpp"

#define ROSROBO_VER 2    // ROSロボのバージョン（第一世代：1、第二世代：2）

/***********************************************************************/
/*                             CountEncoder                            */
/*---------------------------------------------------------------------*/
/*                       エンコーダをカウントする                      */
/***********************************************************************/
// エンコーダのピン
#if ROSROBO_VER == 1     // 第一世代
  #define ENC0A 2
  #define ENC0B 8
  #define ENC1A 3
  #define ENC1B 14
#elif ROSROBO_VER == 2   // 第二世代
  #define ENC0A 7        // PE6(INT6)
  #define ENC0B 6        // PD7
  #define ENC1A A0       // PF7
  #define ENC1B 15       // PB1(PCINT1)
#endif

// エンコーダのカウント値
static int32_t encR, encL;

// 初期設定
void initCountEncoder(void) {
  #if ROSROBO_VER == 1     // 第一世代
  pinMode(ENC0A, INPUT_PULLUP);  // エンコーダ0のAチャンネルを入力に設定
  pinMode(ENC0B, INPUT_PULLUP);  // エンコーダ0のBチャンネルを入力に設定
  pinMode(ENC1A, INPUT_PULLUP);  // エンコーダ1のAチャンネルを入力に設定
  pinMode(ENC1B, INPUT_PULLUP);  // エンコーダ1のBチャンネルを入力に設定
  attachInterrupt(0, interruptEnc0A, FALLING);   // INT0の外部割込みを立下りで設定
  attachInterrupt(1, interruptEnc1A, FALLING);   // INT1の外部割込みを立下りで設定

  #elif ROSROBO_VER == 2   // 第二世代
  pinMode(ENC0A, INPUT);  // エンコーダ0のAチャンネルを入力に設定
  pinMode(ENC0B, INPUT);  // エンコーダ0のBチャンネルを入力に設定
  pinMode(ENC1A, INPUT);  // エンコーダ1のAチャンネルを入力に設定
  pinMode(ENC1B, INPUT);  // エンコーダ1のBチャンネルを入力に設定
  EICRB = (1 << ISC61);          // INT6ピンの下降端で割込み
  EIMSK = (1 << INT6);           // INT6の外部割込み許可
  PCICR = (1 << PCIE0);          // ピン変化割込み許可
  PCMSK0 = (1 << PCINT1);        // PCINT1の割込み許可
  sei();                         // 全割込み許可
  #endif
}

// エンコーダーRの割込み
void interruptEnc0A (void) {
  if (digitalRead(ENC0B) == 0) {
    encR++;
  } else {
    encR--;
  }
}

#if ROSROBO_VER == 1     // 第一世代
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

#elif ROSROBO_VER == 2   // 第二世代
// エンコーダーLの割込み
void interruptEnc1B (void) {
  if (digitalRead(ENC1A) == 0) {
    //encL++;
    encL--;
  } else {
    //encL--;
    encL++;
  }
}

// 外部割込みINT6
ISR(INT6_vect) {
  interruptEnc0A();
}

// ピン変化割込みPCINT0
ISR(PCINT0_vect) {
  static uint8_t lastPin;
  uint8_t presentPin = digitalRead(ENC1B);

  if ((lastPin == HIGH)&&(presentPin == LOW)) {
    // 立下りエッジで実行
    interruptEnc1B();
  }
  lastPin = presentPin;
}
#endif

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
/*                            ブザー関数                               */
/*                          delay()使用禁止！                          */
/***********************************************************************/
/*     PIN設定    */
#define BUZZER_PIN 5         // ブザー出力
// （MIDIのノート番号11～127が使用可能）
#define MIDI_NOTE_MIN   11   // 発音可能なMIDIノート番号の最小値
#define MIDI_NOTE_MAX   127  // 発音可能なMIDIノート番号の最大値
#define PAUSE_NOTE      0xFF // 無音のノート番号

typedef struct note {
  uint8_t midiNumber;   // MIDIノート番号 (59～109が使用可能)
  uint16_t timeLength;  // 音の長さ[ms]
} noteType;

// MIDIのノート番号からOCR3Aの値に変換するテーブル
// IOクロック8MHz、64分周の8ビットタイマカウンタの場合
const uint16_t midi_to_ocr3a[] = { 64934, 60975, 57802, 54347, 51545, 
                                  48543, 45871, 43289, 40815, 38461, 
                                  36363, 34363, 32361, 30580, 28901, 
                                  27247, 25706, 24271, 22882, 21644, 
                                  20407, 19267, 18181, 17152, 16206, 
                                  15290, 14429, 13623, 12852, 12135, 
                                  11454, 10810, 10203, 9633, 9090, 
                                  8583, 8096, 7644, 7214, 6811, 
                                  6426, 6067, 5726, 5404, 5101, 
                                  4814, 4544, 4289, 4049, 3822, 
                                  3607, 3404, 3213, 3033, 2863, 
                                  2702, 2550, 2407, 2272, 2144, 
                                  2024, 1910, 1803, 1702, 1606, 
                                  1516, 1431, 1350, 1275, 1203, 
                                  1135, 1072, 1011, 955, 901, 
                                  850, 803, 757, 715, 675, 
                                  637, 601, 567, 535, 505, 
                                  477, 450, 425, 401, 378, 
                                  357, 337, 318, 300, 283, 
                                  267, 252, 238, 224, 212, 
                                  200, 189, 178, 168, 158, 
                                  149, 141, 133, 126, 118, 
                                  112, 105, 99, 94, 88, 
                                  83, 79 };

//-------------------------------------------------------
// Function : initBuzzer
// 説明     : タイマ0の設定（MIDIのノート番号59～109が使用可能）
//-------------------------------------------------------
static uint8_t defaultTCCR3A;
static uint8_t defaultTCCR3B;
static uint8_t lastTCCR3A;
static uint8_t lastTCCR3B;
static uint8_t lastOCR3A;
static uint8_t lastTCNT3;
static uint8_t usingBuzzer;
void initBuzzer(void) {
  static uint8_t firstFlag;
  // 最初のTCCR3AとTCCR3Bを記憶する
  if (!firstFlag) {
    firstFlag = 1;
    defaultTCCR3A = TCCR3A;
    defaultTCCR3B = TCCR3B;
  }

  pinMode(BUZZER_PIN, OUTPUT);          // ブザーのピンを出力に設定
  TCCR3A = (1 << COM3A0);               // 比較一致でOC0Aﾋﾟﾝ ﾄｸﾞﾙ(交互)出力
  TCCR3B = (1 << WGM32);                // 比較一致ﾀｲﾏ/ｶｳﾝﾀ解除(CTC)動作
  OCR3A = 1;                            // 初期値
  usingBuzzer = 1;
}

// ブザーを一時的に無効化して、delayなどを使えるようにする
void disableBuzzer(void) {
  lastTCCR3A = TCCR3A;
  lastTCCR3B = TCCR3B;
  lastOCR3A = OCR3A;
  lastTCNT3 = TCNT3;
  TCCR3A = defaultTCCR3A;
  TCCR3B = defaultTCCR3B;
  usingBuzzer = 0;
}

// 再度ブザーを有効化する
void enableBuzzer(void) {
  TCCR3A = lastTCCR3A;
  TCCR3B = lastTCCR3B;
  OCR3A = lastOCR3A;
  TCNT3 = lastTCNT3;
  usingBuzzer = 1;
}

//-------------------------------------------------------
// Function : buzzerON
// 説明     : 指定したMIDIノート番号でブザーをスタートさせる
// 引数     : noteNum : MIDIのノート番号
//-------------------------------------------------------
void buzzerON(uint8_t noteNum) {
  if(noteNum == PAUSE_NOTE) {  // 休符が来たとき
    TCCR3B = (1 << WGM32);
    TCNT3 = 0;
  } else if((noteNum >= MIDI_NOTE_MIN) && (noteNum <= MIDI_NOTE_MAX)) {
    OCR3A = midi_to_ocr3a[noteNum - MIDI_NOTE_MIN];
    TCNT3 = 0;
    TCCR3B |= (1 << CS31);  // 64分周でタイマースタート
  }
}

//-------------------------------------------------------
// Function : buzzerOFF
// 説明     : ブザーを終了させる
//-------------------------------------------------------
void buzzerOFF(void) {
  //TCCR3B &= ~((1 << CS32)|(1 << CS31)|(1 << CS30));
  TCCR3B = (1 << WGM32);
  TCNT3 = 0;
}

//-------------------------------------------------------
// 説明     : メロディ関係のグローバル変数
//-------------------------------------------------------
noteType nullNote = {NULL, 0};         // 空の音符
noteType *presentNote = &nullNote;


// 攻撃音案①
noteType shotMelody1[] = {{72, 100},   // ドを200ms
                          {74, 100},   // レを200ms
                          {76, 200},   // ミを500ms
                          {NULL, 0}};  // 必ず最後に必要

//-------------------------------------------------------
// Function : startMelody
// 説明     : 音符配列を鳴動開始
// 引数     : 音符配列の先頭アドレス
//-------------------------------------------------------
void startMelody(struct note *melody) {
  presentNote = melody;
}

//-------------------------------------------------------
// Function : stopMelody
// 説明     : 音符配列の鳴動を止める
//-------------------------------------------------------
void stopMelody(void) {
  presentNote = &nullNote;
}

//-------------------------------------------------------
// Function : driveBuzzer
// 説明     : startMelodyで設定された音符配列を鳴動させる
//            メインルーチンで常時実行すること
// 引数     : なし
// 戻り値   : 0：待機中、1：鳴動中
//-------------------------------------------------------
static uint8_t melodyStep;

uint8_t driveBuzzer(void) {
  //static uint8_t melodyStep;
  static uint32_t startedTime;
  switch(melodyStep) {
    case 0:
      if (presentNote->midiNumber != NULL) { // ノート番号があれば
        buzzerON(presentNote->midiNumber);   // ブザーを鳴らす
        startedTime = getGlobalTime();   // 現在時刻を記録
        melodyStep++;
      } else {
        buzzerOFF();
      }
      break;
    case 1:
      if ((getGlobalTime() - startedTime) >= presentNote->timeLength) {  // length[ms]の時間が経過すれば
        buzzerOFF();
        presentNote++;
        if (presentNote->midiNumber != NULL) { // ノート番号があれば
          buzzerON(presentNote->midiNumber);   // ブザーを鳴らす
          startedTime = getGlobalTime();   // 現在時刻を記録
        } else {
          buzzerOFF();
          melodyStep = 0;
        }
      }
      break;
    default:
      melodyStep = 0;
      break;
  }
  return(melodyStep);
}

//-------------------------------------------------------
// Function : isMelodyFinished
// 説明     : 音符配列の鳴動処理が終了したかどうか知らせる
//-------------------------------------------------------
uint8_t isMelodyFinished(void) {
  return((melodyStep == 0));
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
  
  // ブザーの設定
  initBuzzer();
  startMelody(shotMelody1);
  while(driveBuzzer());
}

void loop() {
  if (updateFlag) {
    updateFlag = 0;
    // Arduino1へ送信
    i2cMasterTransmit(A2state);
    Serial.print(A2state.encR);
    Serial.print(", ");
    Serial.print(A2state.encL);
    Serial.print(", ");
    Serial.println(A2state.time);
  }
}
