class InputCommand {
public:
  float time;
  geometry_msgs::Twist gm;
  float time_sum;
  
  InputCommand(float t,
    double lx=0.0,
    double ly=0.0,
    double lz=0.0,
    double ax=0.0,
    double ay=0.0,
    double az=0.0);
  ~InputCommand(void);
  void setAll(float t,
    double lx=0.0,
    double ly=0.0,
    double lz=0.0,
    double ax=0.0,
    double ay=0.0,
    double az=0.0);
  void setLinear(double lx=0.0, double ly=0.0, double lz=0.0);
  void setAngular(double ax=0.0, double ay=0.0, double az=0.0);
  void reset();
private:

};
