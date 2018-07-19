
typedef struct {
  float time;
  geometry_msgs::Twist gm;
  float time_sum;
} IOrder;

class ControlDrone{
public:
  ControlDrone(void);
  ~ControlDrone(void);
  void drone_proc_loop(void);
  void drone_hover(void);
  void drone_hover2(void);
  void refresh_gm(geometry_msgs::Twist *data);

private:
  // void navinfo_callback(const ardrone_autonomy::Navdata &msg_in);
  void show_navdata(const ardrone_autonomy::Navdata &nav);
  bool check_flight(const uint32_t state);
  void setIOrder(IOrder *iorder,float t,
    double lx=0.0,
    double ly=0.0,
    double lz=0.0,
    double ax=0.0,
    double ay=0.0,
    double az=0.0);

  ros::NodeHandle nh;
  ros::Publisher pub_reset;
  ros::Publisher pub_takeoff;
  ros::Publisher pub_land;
  ros::Publisher pub_cmds;
  ros::Publisher pub_transition;
  ros::Subscriber nav_sub;

  float takeoff_time, avtive_time, landing_time, stop_time;
  bool navinfo_flag;

  geometry_msgs::Twist cmd_msg;
  geometry_msgs::Twist hover_msg;
  std_msgs::Empty empty_msg;
  
};
