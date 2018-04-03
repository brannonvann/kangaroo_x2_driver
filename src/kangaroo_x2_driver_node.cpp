#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <Kangaroo.h>

using namespace std;

class KangarooX2 : public hardware_interface::RobotHW
{
public:
  KangarooX2(string port, unsigned long baud) : 
    serial_port(port, baud, serial::Timeout::simpleTimeout(1000)),
    stream(serial_port),
    K(stream),
    K1(K, '1'),
    K2(K, '2') {

    startChannels();
 
    //Ticks to radians conversion: determined by rotating wheel 10 timees, 
    //then calling 1,getp over simple serial
    //TODO: implement scaling through KangarooChannel::units call
    //  also implement optional reversal of motors
    ticksToRadians = 2*M_PI * 10 / 2995; 
    radiansToTicks = 1/ticksToRadians;
    
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("motor_1_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("motor_2_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("motor_1_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("motor_2_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }

  ~KangarooX2(){
    K1.s(0);
    K2.s(0);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    //send commands to motors
    K1.s((int32_t)  cmd_[0] * radiansToTicks);
    K2.s((int32_t) -cmd_[1] * radiansToTicks);
  }

  void write(){
    KangarooStatus resultGetK1P = K1.getP();
    KangarooStatus resultGetK2P = K2.getP();

    bool allOk = resultGetK1P.ok() && resultGetK2P.ok();
    if (!allOk){
        //assume not ok because channels need to be restarted
        //this happens when kangaroo board is reset
        //try restarting channels
        startChannels(); 
    }
    else { 
        pos_[0] =  resultGetK1P.value() * ticksToRadians;
        pos_[1] = -resultGetK2P.value() * ticksToRadians;
        vel_[0] =  K1.getS().value() * ticksToRadians;
        vel_[1] = -K2.getS().value() * ticksToRadians;
    }
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double ticksToRadians;
  double radiansToTicks;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  serial::Serial serial_port;
  SerialStream stream;
  KangarooSerial  K;
  KangarooChannel K1;
  KangarooChannel K2;

//set up kangaroo channels
void startChannels() {
  ROS_INFO_STREAM("Starting Kangaroo channels");
  K1.start();
  K2.start();
}

};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "kangaroo_x2");
  ros::NodeHandle nh;
  
  //TODO: read port and baud as parameters
  string port = "/dev/ttyUSB0";
  unsigned long baud = 9600;

  KangarooX2 robot(port, baud);
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    //TODO: Detect loss of connection to robot and reconnect
    robot.read();
    robot.write();
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
