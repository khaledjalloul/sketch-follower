#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>

#define MotorInterfaceType 4
#define STEPPER_STEPS 2038

ros::NodeHandle_<ArduinoHardware, 1, 1, 280, 280> nh;
AccelStepper myStepper(MotorInterfaceType, 8, 10, 9, 11);

void subCb(const sensor_msgs::JointState& data){
//for (int i = 0; i < sizeof(data.position); i++){
  for (int i = 0; i < 1; i++){
    
    float goal = floor(- data.position[i] / M_PI * STEPPER_STEPS);
    float currentPos = myStepper.currentPosition();
    
    if (myStepper.currentPosition() < goal -1 || myStepper.currentPosition() > goal + 1)
      myStepper.moveTo(goal);
    
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", &subCb);

void setup()
{
  myStepper.setMaxSpeed(2000.0);
  myStepper.setAcceleration(200.0);
  myStepper.setSpeed(1000);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
  myStepper.run();
}
