#include "InputCommand.hpp"

InputCommand::InputCommand(float t,
  double lx, double ly, double lz,
  double ax, double ay, double az){
  time = t;
  gm.linear.x = lx;
  gm.linear.y = ly;
  gm.linear.z = lz;
  gm.angular.x = ax;
  gm.angular.y = ay;
  gm.angular.z = az;
}

InputCommand::~InputCommand(void){}

void InputCommand::setAll(float t,
  double lx, double ly, double lz,
  double ax, double ay, double az){
  time = t;
  gm.linear.x = lx;
  gm.linear.y = ly;
  gm.linear.z = lz;
  gm.angular.x = ax;
  gm.angular.y = ay;
  gm.angular.z = az;
}

void InputCommand::setLinear(double lx, double ly, double lz){
  gm.linear.x = lx;
  gm.linear.y = ly;
  gm.linear.z = lz;
}

void InputCommand::setAngular(double ax, double ay, double az){
  gm.angular.x = ax;
  gm.angular.y = ay;
  gm.angular.z = az;
}

void InputCommand::reset(void){
  gm.linear.x = 0.0;
  gm.linear.y = 0.0;
  gm.linear.z = 0.0;
  gm.angular.x = 0.0;
  gm.angular.y = 0.0;
  gm.angular.z = 0.0;
}
