#include "ros/ros.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include "plan_reparametrizer/ReparametrizePlan.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

class planReparametrizer{private:
	moveit::planning_interface::MoveGroup group;
	double stitched_velocity_scaling = 1.0;
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	moveit::planning_interface::MoveGroup::Plan plan;
	robot_state::RobotStatePtr rs;

public:
	planReparametrizer():group("robot"){};

	double calcEuclideanDist(std::vector<double> a, std::vector<double> b){
		// ROS_INFO("Entered CED");
		std::vector<double> diff(a.size());

		double sqd_sum = 0;
		for(int i = 0; i < a.size(); i++){
			diff[i] = a[i] - b[i];
			sqd_sum += diff[i]*diff[i];
		}
		return sqrt(sqd_sum);
	}

	bool continuePlan(plan_reparametrizer::ReparametrizePlan::Request & req, plan_reparametrizer::ReparametrizePlan::Response & res) {
		// ROS_INFO("Entered continuePlan");
		// get current joint pose
		std::vector<double> curr_joint_vals = group.getCurrentJointValues();
		rs = group.getCurrentState();
		trajectory_msgs::JointTrajectory tj = req.old_traj;

		double min_dist;
		double min_index;
		// find closest point in traj (in joint space)
		for(int i = 0; i < tj.points.size(); i++){
			std::vector<double> traj_pt = tj.points[i].positions;
			double dist = calcEuclideanDist(curr_joint_vals,traj_pt);

			if(i == 0){
				min_dist = dist;
				min_index = i;
			}
			else{
				if(dist < min_dist){
					min_dist = dist;
					min_index = i;
				}
			}
		}

		// erase already crossed points
		for(int i = 0; i < min_index - 1; i++){
			tj.points.erase(tj.points.begin() + i);
		}
		// put current pose as first point in traj
		tj.points[0].positions = curr_joint_vals;

		plan.trajectory_.joint_trajectory = tj;

		// clear velocities and accelerations
		for(int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
			plan.trajectory_.joint_trajectory.points[i].velocities.clear();
			plan.trajectory_.joint_trajectory.points[i].accelerations.clear();
		}

		rs->setVariablePositions(plan.trajectory_.joint_trajectory.joint_names, plan.trajectory_.joint_trajectory.points.front().positions);
		group.setStartState(*rs);
		robot_trajectory::RobotTrajectory rt(rs->getRobotModel(), group.getName());
		rt.setRobotTrajectoryMsg(*rs, plan.trajectory_);

		// Add velocities to the trajectory using iterative parabolic time parameterization
		iptp.computeTimeStamps(rt, stitched_velocity_scaling);
		rt.getRobotTrajectoryMsg(plan.trajectory_);

		res.new_traj = plan.trajectory_.joint_trajectory;
		ROS_INFO("New plan generated");
		return true;
	}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "plan_reparametrizer");
  ros::NodeHandle n;
  planReparametrizer pr;

  ros::ServiceServer service = n.advertiseService("ReparametrizePlan", &planReparametrizer::continuePlan,&pr);
  ROS_INFO("Plan reparametrizer ready");
  ros::spin();

  return 0;
}
