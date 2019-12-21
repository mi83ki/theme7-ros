/***********************************************************************/
/*                                                                     */
/*  FILE        :arduino1-r05.ino                                      */
/*  DATE        :Nov 19, 2019                                          */
/*  DESCRIPTION :ROSロボ Arduino1用プログラム                          */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/*  メモ：                                                             */
/*        R00：Arduino2との通信テスト                                  */
/*        R01：キーボード操作テスト                                    */
/*        R02：ラズパイとの通信テスト                                  */
/*        R03：Arduino2とI2Cで通信する仕様に変更                       */
/*        R04：Arduino2の通信メッセージにtimeを追加、バンパーtopic追加 */
/*        R05：エンコーダ値のトピックを立てる                          */
/*                                                                     */
/***********************************************************************/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>

#include "comA1andA2.hpp"
#include "pid.hpp"
#include "fix.hpp"
#include "SpeedController.hpp"

/***********************************************************************/
/*                           グローバル変数                            */
/***********************************************************************/
//PID用
static pidType pid_state_right;
static pidType pid_state_left;
static arduino1StateType A1state;

/***********************************************************************/
/*                           モーター関数                              */
/***********************************************************************/
#define DUTY_RESOLUTION 100    // モーター制御のデューティ比の分解能
#define PWM_RESOLUTION 8000    // マイコンのPWM波形1周期の分解能
#define FILT_FREQ 100            //PID制御の周波数[hz] 

/*     PIN設定    */
//#define MOTOR_STBY_PIN 4
#define MOTOR_AIN1_PIN 6
#define MOTOR_AIN2_PIN 7
#define MOTOR_PWMA_PIN 9
#define MOTOR_BIN1_PIN 14
#define MOTOR_BIN2_PIN 8
#define MOTOR_PWMB_PIN 10

#define SEN_LL 5
#define SEN_LC 16
#define SEN_RC 2
#define SEN_RR 4
#define SEN_TR 15

// モーターの初期化
void initMotorsAndTimer1(void) {
  //------------------------
  //  PIN設定
  //------------------------
  //pinMode(MOTOR_STBY_PIN, OUTPUT );
  pinMode(MOTOR_AIN1_PIN, OUTPUT );
  pinMode(MOTOR_AIN2_PIN, OUTPUT );
  pinMode(MOTOR_PWMA_PIN, OUTPUT );
  pinMode(MOTOR_BIN1_PIN, OUTPUT );
  pinMode(MOTOR_BIN2_PIN, OUTPUT );
  pinMode(MOTOR_PWMB_PIN, OUTPUT );

  //------------------------
  //  出力初期化
  //------------------------
  //digitalWrite(MOTOR_STBY_PIN, LOW);  // STBY出力を0に
  digitalWrite(MOTOR_AIN1_PIN, LOW);  // A1出力を0に
  digitalWrite(MOTOR_AIN2_PIN, LOW);  // A2出力を0に
  //analogWrite(MOTOR_PWMA_PIN, 0);  // PWMA出力を0に
  digitalWrite(MOTOR_BIN1_PIN, LOW);  // B1出力を0に
  digitalWrite(MOTOR_BIN2_PIN, LOW);  // B2出力を0に
  //analogWrite(MOTOR_PWMB_PIN, 0);  // PWMB出力を0に

  //------------------------
  //  タイマカウンタ1の設定
  //------------------------
  TCCR1A = (1 << COM1A1) | (1 << COM1B1); // 比較一致でLow、BOTTOMでHighをOC1xﾋﾟﾝへ出力 (非反転動作)
  TCCR1A |= (1 << WGM11);               // 高速PWM動作（TOP値：ICR1）
  TCCR1B = (1 << WGM13) | (1 << WGM12); // 高速PWM動作（TOP値：ICR1）
  OCR1A = PWM_RESOLUTION;               // 初期値
  OCR1B = PWM_RESOLUTION;               // 初期値
  ICR1 = 7999;                          // 8MHzクロック前置分周なしで1kHzのPWM波形＆割込み生成
  //TIMSK1 = (1 << ICIE1);                // ﾀｲﾏ/ｶｳﾝﾀ1捕獲割り込み許可
  //sei();                                // 全割込み許可
  TCCR1B |= (1 << CS10);                // 前置分周なしでタイマースタート
}

//-------------------------------------------------------
// Function : driveMotors
// 引数     : dutyR : 右モーターの電圧[％]（-100%～100%）
//            dutyL : 左モーターの電圧[％]（-100%～100%）
//-------------------------------------------------------
void driveMotors(int32_t dutyR, int32_t dutyL) {
  //------------------------
  // 右モーター
  //------------------------
  if (dutyR > 0) {                              // 正転させるとき
    //digitalWrite(MOTOR_AIN1_PIN, HIGH);         //通常版
    //digitalWrite(MOTOR_AIN2_PIN, LOW);          // CW
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);          // CW
  } else if (dutyR == 0) {
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, LOW);          // フリーストップ
  } else {                                      // 逆転させるとき
    dutyR *= -1;
    //digitalWrite(MOTOR_AIN1_PIN, LOW);           //通常版
    //digitalWrite(MOTOR_AIN2_PIN, HIGH);          // CCW
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, LOW);         // CCW
  }
  if (dutyR > DUTY_RESOLUTION) {                // 指定値がオーバーフローしているとき
    dutyR = DUTY_RESOLUTION;
  } else if (dutyR > 0) {
    dutyR *= PWM_RESOLUTION / DUTY_RESOLUTION;  // PWMの分解能領域に投影
    OCR1A = dutyR - 1;                          // PWM波形の設定
  } else {
    OCR1A = PWM_RESOLUTION;
  }

  //------------------------
  // 左モーター
  //------------------------
  if (dutyL > 0) {                              // 正転させるとき
    digitalWrite(MOTOR_BIN1_PIN, HIGH);
    digitalWrite(MOTOR_BIN2_PIN, LOW);          // CW
  } else if (dutyL == 0) {
    digitalWrite(MOTOR_BIN1_PIN, LOW);
    digitalWrite(MOTOR_BIN2_PIN, LOW);          // フリーストップ
  } else {                                      // 逆転させるとき
    dutyL *= -1;
    digitalWrite(MOTOR_BIN1_PIN, LOW);
    digitalWrite(MOTOR_BIN2_PIN, HIGH);         // CCW
  }
  if (dutyL > DUTY_RESOLUTION) {                // 指定値がオーバーフローしているとき
    dutyL = DUTY_RESOLUTION;
  } else if (dutyL > 0) {
    dutyL *= PWM_RESOLUTION / DUTY_RESOLUTION;  // PWMの分解能領域に投影
    OCR1B = dutyL - 1;                          // PWM波形の設定
  } else {
    OCR1B = PWM_RESOLUTION;
  }

  //digitalWrite(MOTOR_STBY_PIN, HIGH);           // STBY出力を1にしてモーター有効
}

// モーターを止めたいとき
void stopMotors(void) {
  OCR1A = PWM_RESOLUTION;
  OCR1B = PWM_RESOLUTION;
  digitalWrite(MOTOR_BIN1_PIN, LOW);
  digitalWrite(MOTOR_BIN2_PIN, LOW);            // フリーストップ
  //digitalWrite(MOTOR_STBY_PIN, LOW);            // STBY出力を0にしてモーター無効
}


/***********************************************************************/
/*                           タイマー関数                              */
/*        注：initMotor()によって1msの割込みが設定されている           */
/***********************************************************************/
static uint32_t tempTimer;  // 1msの割り込みでインクリメントされる変数

// タイマーをクリアする
void startTimer(void) {
  tempTimer = millis();
}

// タイマー値[ms]を取得する
uint32_t getTime(void) {
  return (millis() - tempTimer);
}

// タイマー値[ms]を取得する
uint32_t getGlobalTime(void) {
  return (millis());
}

/***********************************************************************/
/*                           バンパー関数                              */
/***********************************************************************/

void initBumper(void) {
  pinMode(SEN_LL, INPUT);
  pinMode(SEN_LC, INPUT);
  pinMode(SEN_RC, INPUT);
  pinMode(SEN_RR, INPUT);
  pinMode(SEN_TR, OUTPUT);
  digitalWrite(SEN_TR, HIGH);
}

uint8_t readBumper(void) {
  uint8_t returnData = 0;

  returnData |= (digitalRead(SEN_LL) << 3);
  returnData |= (digitalRead(SEN_LC) << 2);
  returnData |= (digitalRead(SEN_RC) << 1);
  returnData |= (digitalRead(SEN_RR) << 0);

  return (returnData);
}

// 引数のバンパー値を更新した上で、前回から変化したら1を、そうでなければ0を返す
uint8_t isBumperChanged(uint8_t *presentBumper) {
  static uint8_t lastBumper;    // バンパーの前回値

  *presentBumper = readBumper();
  if (*presentBumper != lastBumper) {
    lastBumper = *presentBumper;
    return (1);
  } else {
    return (0);
  }
}

/***********************************************************************/
/*                               ROS関数                               */
/***********************************************************************/
ros::NodeHandle nh;

//-------------------------------------------------------
//             cmd_velのサブスクライバー
//-------------------------------------------------------
#define WHEEL_TRACK 0.0625f // 車輪左右間隔[m]
#define MAX_VELOCITY 0.7f // デューティ100％の時の速度[m/s]
#define R_MOTOR_SPEC 1.0f // モーターの固有ばらつき補正値
#define L_MOTOR_SPEC 1.0f // モーターの固有ばらつき補正値
//#define L_MOTOR_SPEC 0.98f
#define DUTY_MIN 15       // モーターが回転し始める最小のDUTY比

void messageCb2(const geometry_msgs::Twist& twist) {
  const float linear_x = twist.linear.x;
  const float angular_z = twist.angular.z;
  //int32_t linear_x = (int32_t)twist.linear.x;
  //int32_t angular_z = (int32_t)twist.angular.z;
  int32_t motorR = 0;
  int32_t motorL = 0;
  float velocityR = linear_x - WHEEL_TRACK * angular_z;
  float velocityL = linear_x + WHEEL_TRACK * angular_z;

  if (velocityR > MAX_VELOCITY) {
    velocityR = MAX_VELOCITY;
  } else if (velocityR < -MAX_VELOCITY) {
    velocityR = -MAX_VELOCITY;
  }
  if (velocityL > MAX_VELOCITY) {
    velocityL = MAX_VELOCITY;
  } else if (velocityL < -MAX_VELOCITY) {
    velocityL = -MAX_VELOCITY;
  }

  //pid_state_right.desired = - FLOAT_TO_FIX(velocityR);
  //pid_state_left.desired =  FLOAT_TO_FIX(velocityL);

  motorR = (int32_t)(velocityR / MAX_VELOCITY * 100.0 * R_MOTOR_SPEC);
  motorL = (int32_t)(velocityL / MAX_VELOCITY * 100.0 * L_MOTOR_SPEC);

  /*
    // 0 ～ DUTY_RESOLUTIONの領域をDUTY_MIN ～ DUTY_RESOLUTION の領域に投影
    if (motorR > 0) {
      motorR = DUTY_MIN + motorR * (DUTY_RESOLUTION - DUTY_MIN) / DUTY_RESOLUTION;
    } else if (motorR < 0) {
      motorR = -DUTY_MIN + motorR * (DUTY_RESOLUTION - DUTY_MIN) / DUTY_RESOLUTION;
    }
    if (motorL > 0) {
      motorL = DUTY_MIN + motorL * (DUTY_RESOLUTION - DUTY_MIN) / DUTY_RESOLUTION;
    } else if (motorL < 0) {
      motorL = -DUTY_MIN + motorL * (DUTY_RESOLUTION - DUTY_MIN) / DUTY_RESOLUTION;
    }
  */
  //driveMotors(motorR, motorL);
}

// cmd_velを取得するサブスクライバー
ros::Subscriber<geometry_msgs::Twist> sub2("arduino_cmd_vel", &messageCb2);

//-------------------------------------------------------
//         バンパーステータスののパブリッシャー
//-------------------------------------------------------
// バンパートピック
std_msgs::UInt8 bumper_msg;
ros::Publisher pubBumper("Bumper", &bumper_msg);

//-------------------------------------------------------
//         エンコーダカウントのパブリッシャー
//-------------------------------------------------------
// エンコーダデータ
std_msgs::Int32 encR_msg;
std_msgs::Int32 encL_msg;
std_msgs::UInt32 time_msg;

//Publisherのインスタンス
//トピック名：encR, encL, time_A2
ros::Publisher pub3("A2_encR", &encR_msg);
ros::Publisher pub4("A2_encL", &encL_msg);
ros::Publisher pub5("A2_time", &time_msg);

/***********************************************************************/
/*                               main関数                              */
/***********************************************************************/
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // モーターの設定
  initMotorsAndTimer1();
  // バンパーの設定
  initBumper();
  // Arduino2とのI2C通信設定
  initComA1andA2(SLAVE);

  initPID(&pid_state_right, KTT, &A1state.vel_right, FLOAT_TO_FIX(0.5), 0.3, 1.5, 0.0);
  initPID(&pid_state_left, KTT, &A1state.vel_left, FLOAT_TO_FIX(0.5), 0.3, 1.5, 0.0);
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  uint8_t bumperStatus = 0;
  static uint32_t startedTime;
  static uint8_t encUpdFlag = 0;
  static uint8_t encUpdFlag2 = 0;
  // Arduino2の状態量
  static arduino2StateType A2state;
  SpeedControll spdControll;        //速度制御インスタンス生成
  fix power_right = 10;
  fix power_left = 10;
  pid_state_right.desired = FLOAT_TO_FIX(0.5);
  pid_state_left.desired = FLOAT_TO_FIX(0.5);

  // エンコーダ値更新処理
  if (isI2Crecieved()) {
    i2cSlaveRecieve(&A2state);
    encUpdFlag++;
    encUpdFlag2++;
  }


  // 速度の計算
  if (encUpdFlag2) {
    encUpdFlag2 = 0;
    spdControll.calcSpeed(A2state, &A1state);        //現在速度の計算
    power_right = pidControl(&pid_state_right, FILT_FREQ);
    power_left = pidControl(&pid_state_left, FILT_FREQ);
    //Serial.print("power:");
    //Serial.print(FIX_TO_FLOAT(power_right));
    //Serial.print(',');
    //Serial.print(FIX_TO_FLOAT(power_left));
    //Serial.print(',');
    //Serial.print("time:");
    //Serial.print(A2state.time);
    //Serial.print(',');
    
//    fixcutoff(&power_right, INT_TO_FIX(100), -INT_TO_FIX(100));
//    fixcutoff(&power_left, INT_TO_FIX(100), -INT_TO_FIX(100));

    spdControll.controllMotorsSpeed(power_right, power_left, A1state);
    driveMotors(spdControll.duty_status.duty_right, spdControll.duty_status.duty_left);
    //driveMotors(100, 100);
    /*Serial.print("aim_vel,");
      Serial.print(FIX_TO_FLOAT(pid_state_right.desired));
      Serial.print(',');
      Serial.print(FIX_TO_FLOAT(pid_state_left.desired));
      Serial.print("vel,");
      Serial.print(FIX_TO_FLOAT(*pid_state_right.present));
      Serial.print(',');
      Serial.println(FIX_TO_FLOAT(*pid_state_left.present)); */
  }


  /*
    // ROSへの周期的なパブリッシュ
    if (getTime() >= 50) {
      startTimer();

      if (!nh.connected()) {   // rosserialが切れたら、再接続する
        //nh.getHardware()->setBaud(74880);
        nh.initNode();
        nh.subscribe(sub2);
        nh.advertise(pubBumper);
        nh.advertise(pub3);
        nh.advertise(pub4);
        nh.advertise(pub5);
        //Serial.print("reconnecting");
        while (!nh.connected()) {
          //Serial.print(".");
          nh.spinOnce();
          delay(1000);
        }
        //Serial.println("Connect.");
      } else {                    // rosserialが接続しているときの動作
        // バンパー値をパブリッシュ
        if (isBumperChanged(&bumperStatus)) {
          bumper_msg.data = bumperStatus;
          pubBumper.publish(&bumper_msg);
        }

        // エンコーダ値をパブリッシュ
        if (encUpdFlag) {
          encUpdFlag = 0;
          encR_msg.data = A2state.encR;
          encL_msg.data = A2state.encL;
          time_msg.data = A2state.time;
          pub3.publish(&encR_msg);
          pub4.publish(&encL_msg);
          pub5.publish(&time_msg);
        }
      }
    }
    nh.spinOnce();
  */
}


