#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <control_drone/Transition.h>

#include <iostream>
#include "ardrone_test.hpp"
#include "InputCommand.cpp"

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
: takeoff_time(5.0),avtive_time(4.0),landing_time(3.0),navinfo_flag(false)
{
  pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pub_reset = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  pub_cmds = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pub_transition = nh.advertise<control_drone::Transition>("/transition", 1);
  // nav_sub = nh.subscribe("/ardrone/navdata", 1, &ControlDrone::navinfo_callback, this)

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
  cmd_msg.angular.z = -0.1;
}

ControlDrone::~ControlDrone(){}

// データのリフレッシュ
void ControlDrone::refresh_gm(geometry_msgs::Twist *data){
  data->linear.x = 0.0;
  data->linear.y = 0.0;
  data->linear.z = 0.0;
  data->angular.x = 0.0;
  data->angular.y = 0.0;
  data->angular.z = 0.0;
}

void ControlDrone::drone_proc_loop(void){
  double time = (double)ros::Time::now().toSec();
  ROS_INFO("Starting ARdrone_fly loop");

  float t1 = takeoff_time;
  float t2 = avtive_time + t1;
  float t3 = landing_time + t2;

  ros::Rate loop_rate(50);
  while (ros::ok()){
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

void ControlDrone::setIOrder(IOrder *iorder,float t,
  double lx, double ly, double lz,
  double ax, double ay, double az){
  iorder->time = t;
  iorder->gm.linear.x = lx;
  iorder->gm.linear.y = ly;
  iorder->gm.linear.z = lz;
  iorder->gm.angular.x = ax;
  iorder->gm.angular.y = ay;
  iorder->gm.angular.z = az;
}

void ControlDrone::drone_hover2(void){
  control_drone::Transition cdtr;
  double time = (double)ros::Time::now().toSec();
  ROS_INFO("Starting ARdrone_fly mode = HOVER2");
  InputCommand iorder[2] = { //プロセスの内容
		InputCommand(5.0),
		InputCommand(3.0,0.1)
	};
  int i = 0,j = 1;    // 現在のプロセス
  int i_end = 1;      // プロセスの終了地点 設定値
  cdtr.transition = -1;

  // 時間の計算
  float t_takeoff = 5.0;    //離陸時間
  iorder[0].time_sum = iorder[0].time + t_takeoff;
  for(j=1; j<i_end+1; j++){
    iorder[j].time_sum = iorder[j].time + iorder[j-1].time_sum;
  }
  float t_land = 3.0 + iorder[i_end].time_sum;  //着陸時間

  //std::cout << "出力時間 = " << iorder[0].time << std::endl;

  ros::Rate loop_rate(50);
  while (ros::ok()){
    double now = (double)ros::Time::now().toSec();
    if(now < time+t_takeoff){ //離陸
      pub_takeoff.publish(empty_msg);
      ROS_INFO_ONCE("Taking off");
      //pub_cmds.publish(hover_msg);
    }else if(now > time + t_takeoff && now < time + iorder[i].time_sum){ //コマンド命令  入れる
      pub_cmds.publish(iorder[i].gm);
      ROS_INFO_ONCE("Moving/Hovering");
      cdtr.transition = i;
    }else if(now > time+iorder[i_end].time_sum && now < time+t_land){ //着陸
      pub_land.publish(empty_msg);
      ROS_INFO_ONCE("Landing");
      cdtr.transition = 10;
    }else if(now > time+t_land){ //終了
      ROS_INFO_ONCE("Closing Node");
      pub_reset.publish(empty_msg);//Droneのリセット
      cdtr.transition = 11;
      break;
    }
    if (i < i_end && ros::Time::now().toSec() > time + iorder[i].time_sum){
      i++;
      std::cout << "数値が " << i << " に変更されました" << std::endl;
    }
    pub_transition.publish(cdtr);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void ControlDrone::drone_hover(void){
  double time = (double)ros::Time::now().toSec();
  ROS_INFO("Starting ARdrone_fly mode = HOVER");
  IOrder iorder[2];   // プロセスの内容
  int i = 0,j = 1;    // 現在のプロセス
  int i_end = 1;      // プロセスの個数 設定値

  // iorderのセットアップ
  setIOrder(&iorder[0],5.0);
  setIOrder(&iorder[1],3.0,1.0);


  // 時間の計算
  float t_takeoff = 5.0;    //離陸時間
  iorder[0].time_sum = iorder[0].time + t_takeoff;
  for(j=1; j<i_end+1; j++){
    iorder[j].time_sum = iorder[j].time + iorder[j-1].time_sum;
  }
  float t_land = 3.0 + iorder[i_end].time_sum;  //着陸時間

  //std::cout << "出力時間 = " << iorder[0].time << std::endl;

  ros::Rate loop_rate(50);
  while (ros::ok()){
    double now = (double)ros::Time::now().toSec();
    if(now < time+t_takeoff){ //離陸
      pub_takeoff.publish(empty_msg);
      ROS_INFO_ONCE("Taking off");
      //pub_cmds.publish(hover_msg);
    }else if(now > time + t_takeoff && now < time + iorder[i].time_sum){ //コマンド命令
      pub_cmds.publish(iorder[i].gm);
      pub_cmds.publish(hover_msg);
      ROS_INFO_ONCE("Moving/Hovering");
      if (ros::Time::now().toSec() > time + iorder[i].time_sum){
        i++;
      }
    }else if(now > time+iorder[i_end].time_sum && now < time+t_land){ //着陸
      pub_land.publish(empty_msg);
      ROS_INFO_ONCE("Landing");
    }else if(now > time+t_land){ //終了
      ROS_INFO_ONCE("Closing Node");
      pub_reset.publish(empty_msg);//Droneのリセット
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool ControlDrone::check_flight(uint32_t state) {
  std::string drone_states[]={"UNKNOWN","INIT","LANDED","FLYING1",
  "HOVERING","TEST","TAKEOFF","FLYING2","LANDING","LOOPING"};

  ROS_INFO_STREAM("Drone State : " << drone_states[state]);
  if(state == SD_FLYING1 || state == SD_HOVERING || state == SD_FLYING2){
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
  d.drone_hover2();
  return 0;
}
