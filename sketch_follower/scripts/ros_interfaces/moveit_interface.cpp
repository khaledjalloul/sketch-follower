#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose2D.h>
#include <string>

using namespace std;

class MoveitInterface
{
private:
	const double jump_threshold = 0.0;
	const double eef_step = 1;

	moveit::planning_interface::MoveGroupInterface *move_group_interface;
	geometry_msgs::Pose cartesian_target;
	vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	double fraction;

	bool move_robot = false;

	void cursorCb(const geometry_msgs::Pose2D::ConstPtr &msg)
	{
		cout << msg->x << " " << msg->y << endl;
		cartesian_target.position.y = msg->x;
		cartesian_target.position.x = msg->y;
		waypoints.push_back(cartesian_target);
	}

	void moveToHome()
	{
		cartesian_target.position.x = 0.5;
		cartesian_target.position.y = 1.8;
		waypoints.push_back(cartesian_target);

		fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

		move_group_interface->execute(trajectory);

		waypoints.clear();
	}

public:
	MoveitInterface(ros::NodeHandle &n)
	{
		ros::AsyncSpinner spinner(1);
		spinner.start();

		// const string PLANNING_GROUP = "robot_arm_joints";
		const string PLANNING_GROUP = "Joints";
		move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
		const moveit::core::JointModelGroup *joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		ROS_INFO("Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
		ROS_INFO("End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

		cartesian_target = move_group_interface->getCurrentPose().pose;

		// moveToHome();

		ros::Rate r(20);
		ros::Subscriber sub = n.subscribe<geometry_msgs::Pose2D>("/sketch_follower/cursor_position", 10, &MoveitInterface::cursorCb, this);
		ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("/sketch_follower/eef_position", 10);

		geometry_msgs::Pose2D eef_position;

		while (ros::ok())
		{
			if (n.getParam("move_robot", move_robot) && move_robot)
			{
				fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
				ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
				move_group_interface->stop();

				waypoints.clear();
				move_group_interface->asyncExecute(trajectory);
				move_robot = false;
				n.setParam("move_robot", false);
			}

			eef_position.x = move_group_interface->getCurrentPose().pose.position.x;
			eef_position.y = move_group_interface->getCurrentPose().pose.position.y;
			pub.publish(eef_position);
			r.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveit_cpp");
	ros::NodeHandle n;

	MoveitInterface interface(n);
	return 0;
}
