#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"

#define GET_ARRAY_SIZE(a)   (sizeof(a)/sizeof(a[0]))//

nav_msgs::Odometry input_msg;//subscribeしてくるpose型のメッセージを定義
geometry_msgs::Twist output_msg;//publish message

float kp1 = 0; //P制御の定数
float kp2 = 0; //P制御の定数2
float angular_n = 0;
int point_id = 0;
int mode = 0; //0…角度修正モード 1…走行モードt
float point_array[5][2] = {{0,0},{0.9,0},{0.9,0.9},{0,0.9},{0,0}};//pass point array {x,y}

float calc_angle(const float x,const float xn1,const float y,const float yn1);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   input_msg = *msg;
 //  ROS_INFO("Pose: [%f],[%f],[%f]", msg->x,msg->y,msg->theta);
}

int main(int argc, char **argv)
{
  // 初期化
    //現在位置X,Y,角度θをもらう
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("odm_cal", 10, poseCallback); //角度，速度を読んでくるsubscriberを定義
        ros::Publisher cmd_pub= n.advertise<geometry_msgs::Twist>("arduino_cmd_vel",1000);//角度,速度を配信するpublisherを定義
        ros::Rate loop_rate(10); 

        while (ros::ok())
        {
            if( point_id <= GET_ARRAY_SIZE(point_array) -1 ){https://kazuhira-r.hatenablog.com/entry/20180728/1532770315
                nav_msgs::Odometry pose_msg = input_msg;//publishするtwist型のメッセージを定義
                float xn = point_array[point_id][0];
                float yn = point_array[point_id][1];
                switch(mode){
                   case 0:
                    output_msg.angular.z = 0.5;
                    angular_n = calc_angle(pose_msg.pose.pose.position.x,xn,pose_msg.pose.pose.position.y,yn);
                    ROS_INFO("I want to go xn:[%f] yn:[%f]",xn,yn);
                    ROS_INFO("x:[%f] y:[%f]",pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y);
                    ROS_INFO("mode:[%d] theta:[%f] angular_n:[%f] abs(pose_msg.theta - angular_n):[%f]",mode,pose_msg.twist.twist.angular.z ,angular_n,std::abs(pose_msg.twist.twist.angular.z - angular_n));
                    if(std::abs(pose_msg.twist.twist.angular.z - angular_n) < 0.05 )
                    {

                      mode = 1;
                      output_msg.angular.z = 0;

                    }
                    break;
                   case 1:
                    output_msg.linear.x = 1;
                    if((std::abs(pose_msg.pose.pose.position.x -xn) < 0.1) && (std::abs(pose_msg.pose.pose.position.y -yn) < 0.1))
                    {
                       mode = 0;
                       output_msg.linear.x = 0;
                       point_id += 1;
                    }
                    break;
                   default:
                    break;
                }
                cmd_pub.publish(output_msg);
            }else{

            }

            ros::spinOnce();
            loop_rate.sleep();

        } 
  return 0;
}

//float calc_omega_p(float x,float xn,float y,float yn,float theta){
// float omega_p;
// omega_p = -kp1*(atan((yn - y)/(xn - x)) - theta);
// return omega_p;
//}
 //ωpを計算する関数

float calc_angle(const float x,const float xn1,const float y,const float yn1){ //calculate theta to Goal
    float theta;
    theta = std::atan2((yn1 - y),(xn1 - x));
    if(theta < 0){
        theta = theta + 2*M_PI;
    }
    return theta;
}
