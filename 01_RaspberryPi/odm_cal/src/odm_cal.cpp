//#include "odm_cal.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "bits/stdc++.h"

#define WHEEL_R         (0.090)       /* 車輪半径(m) */
#define WHEEL_TREAD     (0.125)       /* 車輪間距離(m) */
#define RATE_REDC       (298.0)       /* 減速比 */
#define RATE_ENC        (3.0)         /* モータ1回転に必要なエンコーダカウント数 */


/* debug Mode */
#define DEBUG
#define DEBUG_SUBSC

typedef struct{
    double x;
    double y;
    double th;

    double v;
    double omega;
    //float x;
    //float y;
    //float th;

    //float v;
    //float omega;
}stateType;

typedef struct{
public:
    double enc_L;
    double enc_R;
    //float enc_L;
    //float enc_R;
}encType;

class OdmPublisher
{
public:
  void CalOdm();
  //void ~CalOdm();
  void InitOdm();

  ros::Subscriber enc_sub;
  ros::Subscriber enc_time_sub;

protected:
  //ros::Subscriber sub_gazebo;   ///<gazebo_msgs入力ハンドラ
  //ros::Publisher pub_pose;      ///<ロボット位置Pose 発行ハンドラ

  ros::Time cuurent_time;
  ros::Publisher odm_pub;


#ifdef DEBUG
  double enc_L;
  double enc_R;

  double pre_enc_L;
  double pre_enc_R;

  double time;
  double pre_time;

#else

#endif

  stateType state;
  stateType pre_state;
  encType   delta_enc;
};

void OdmPublisher::InitOdm()
{
#ifdef DEBUG
//  enc_L = 1311;
//  enc_R = 1311;     /* 310.41 */

  enc_L = 1000;
//  enc_R = 1894;     /* 310.41 */
  enc_R = 1010;

  pre_enc_L = 1000;
  pre_enc_R = 1000;

  time      = 100010;
  pre_time  = 100000;

#else

#endif
  ros::NodeHandle n;
  odm_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ROS_INFO("[odm_cal] init");

}

void OdmPublisher::CalOdm()
{
  cuurent_time = ros::Time::now();

#ifdef DEBUG
  InitOdm();
#endif

  /* エンコーダ加算値とサンプリング時間を計算 */
  delta_enc.enc_L = enc_L - pre_enc_L;
  delta_enc.enc_R = enc_R - pre_enc_R;
  double delta_t = (time - pre_time) * 0.001;

  pre_time = time;

  /* 車輪の速度を計算 */
  double wheel_v_L = (2 *M_PI *WHEEL_R *delta_enc.enc_L) / (RATE_ENC *RATE_REDC *delta_t);
  double wheel_v_R = (2 *M_PI *WHEEL_R *delta_enc.enc_R) / (RATE_ENC *RATE_REDC *delta_t);

  /* 車体の速度と回転角速度を計算 */
  state.v = (wheel_v_L + wheel_v_R)/2;
  state.omega = (wheel_v_R - wheel_v_L)/(WHEEL_TREAD); //WHEEL_Tトレッド

  /* 位置座標を計算 */
  double delta_th = delta_t * (state.omega + pre_state.omega)/2;
  state.th += delta_th;

  double delta_x = delta_t * (state.v * cos(state.th) + pre_state.v * cos(pre_state.th))/2;
  double delta_y = delta_t * (state.v * sin(state.th) + pre_state.v * sin(pre_state.th))/2;

  state.x += delta_x;
  state.y += delta_y;

  ROS_INFO("[odm_cal] dt = %lf, encL = %lf, encR = %lf", delta_t, delta_enc.enc_L, delta_enc.enc_R);

  ROS_INFO("[odm_cal] v = %lf, omega = %lf", state.v, state.omega);

  ROS_INFO("[odm_cal] (dx, dy, dth) = (%lf, %lf ,%lf)", delta_x, delta_y, delta_th);
  ROS_INFO("[odm_cal] (x, y, th)    = (%lf, %lf ,%lf)", state.x, state.y, state.th);

  nav_msgs::Odometry odm;

//  odm.header.stamp = cuurent_time;
//  odm.header.frame_id = "odm";

  odm.pose.pose.position.x = state.x;
  odm.pose.pose.position.y = state.y;
  odm.pose.pose.position.z = 0.0;
  //odm.pose.pose.orientation.w = state.th;

  odm.twist.twist.linear.x = state.v * cos(state.th);
  odm.twist.twist.linear.y = state.v * sin(state.th);
  odm.twist.twist.linear.z = 0.0;

  odm.twist.twist.angular.x = 0.0;
  odm.twist.twist.angular.y = 0.0;
  odm.twist.twist.angular.z = state.omega;

  odm_pub.publish(odm);

  pre_state = state;
}

/*
void OdmPublisher::~CalOdm()
{

}  ros::Publisher OdmPub.odm_pub = n.advertise<nav_msgs::Odometry>("odm", 50);

*/

int main(int argc, char **argv)
{
  int cnt = 0;

  ros::init(argc, argv, "odm_cal");


  OdmPublisher OdmPub;
  ros::NodeHandle n;

  ros::Rate r(10);

  OdmPub.InitOdm();
  ROS_INFO("[odm_cal] START");
//  ros::Subscriber enc_sub = n.subscribe("enc", 50, OdmPub.CalOdm);
//  ros::spin();

 // /*
  while (ros::ok())
  {
    OdmPub.CalOdm();

    ++cnt;
    ROS_INFO("[odm_cal] %d", cnt);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
 // */
}
