#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include "ardrone_test.hpp"

//ドローンの状態(State of Drone)
#define SD_UNKNOWN 0
#define SD_INIT 1
#define SD_LANDED 2
#define SD_FLYING1 3
#define SD_HOVERING 4
#define SD_TEST 5
#define SD_TAKEOFF 6
#define SD_FLYING2 7
#define SD_LANDING 8
#define SD_LOOPING 9

ControlDrone::ControlDrone()
: takeoff_time(5.0),avtive_time(10.0),landing_time(3.0),navinfo_flag(false)
{
  pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pub_reset = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  pub_cmds = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  nav_sub = nh.subscribe("/ardrone/navdata", 1, &ControlDrone::navinfo_callback, this);

  //ホバリング制御用メッセージ
  hover_msg.linear.x = 0.0;
  hover_msg.linear.y = 0.0;
  hover_msg.linear.z = 0.0;
  hover_msg.angular.x = 0.0;
  hover_msg.angular.y = 0.0;
  hover_msg.angular.z = 0.0;

  //フライト制御用メッセージ
  cmd_msg.linear.x = 0.0;
  cmd_msg.linear.y = 0.0;
  cmd_msg.linear.z = 0.0;
  cmd_msg.angular.x = 0.0;
  cmd_msg.angular.y = 0.0;
  cmd_msg.angular.z = 2.5;
}

ControlDrone::~ControlDrone(){}

void ControlDrone::drone_proc_loop(void){
  double time = (double)ros::Time::now().toSec();
  ROS_INFO("Starting ARdrone_fly loop");

  float t1 = takeoff_time;
  float t2 = avtive_time + t1;
  float t3 = landing_time + t2;

  ros::Rate loop_rate(50);
  while (ros::OK()){
    double now = (double)ros::Time::now().toSec();
    if(now < time+t1){
      //離陸
      pub_takeoff.publish(empty_msg);
      ROS_INFO_ONCE("Taking off");
      pub_cmds.publish(hover_msg);
    }else if(now > time+t1 && now < time+t2){
      //回転
      pub_cmds.publish(cmd_msg);
      ROS_INFO_ONCE("Movinf/Hovering");
      navinfo_flag = true;
    }else if(now > time+t2 && now < time+t3){
      //着陸
      pub_land.publish(empty_msg);
      ROS_INFO_ONCE("Landing");
    }else if(now > time+t3){
      //終了
      ROS_INFO_ONCE("Closing Node");
      //Droneのリセット
      pub_reset.publish(empty_msg);
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool ControlDrone::check_flight(unit32_t state) {
  std::string drone_states[]={"UNKNOWN","INIT","LANDED","FLYING1",
  "HOVERING","TEST","TAKEOFF","FLYING2","LANDING","LOOPING"}

  ROS_INFO_STREAM("Drone State : " << drone_states[state]);
  if(state == FLYING1 || state == HOVERING || state==FLYING2){
    return true;
  }else{
    return false;
  }
}

void ControlDrone::show_navdata(const ardrone_autonomy::Navdata& nav){
  std::cout <<  "header : " << std::endl;
  std::cout <<  " " << nav.header;
  std::cout << "batteryPercent : " << nav.batteryPercent << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"ARDrone_flight");
  ControlDrone d;
  d.drone_proc_loop();
  return 0;
}
