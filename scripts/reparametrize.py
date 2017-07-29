#!/usr/bin/env python

import sys
import numpy
import rospy
import math
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from plan_reparametrizer.srv import ReparametrizePlanPy

class plan_reparametrizer:
    velocity_scaling = 1.0

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("robot")
        self.robot= moveit_commander.RobotCommander()

    def calc_euclidean_dist(self,curr_pose, traj_point_pose):
        diff = [0]*len(curr_pose)
        sqdsum = 0
        for i in range(len(curr_pose)):
            diff[i] = curr_pose[i] - traj_point_pose[i]
            sqdsum = sqdsum + diff[i]*diff[i]
        return math.sqrt(sqdsum)

    def continue_plan(self,req):
        curr_pose = self.group.get_current_joint_values()

        for i in range(len(req.old_plan.joint_trajectory.points)):
            dist = self.calc_euclidean_dist(curr_pose,req.old_plan.joint_trajectory.points[i].positions)
            if i == 0:
                min_dist = dist
                min_index = i
            elif dist < min_dist:
                min_dist = dist
                min_index = i

        edited_plan = RobotTrajectory()
        edited_plan.joint_trajectory.header = req.old_plan.joint_trajectory.header
        edited_plan.joint_trajectory.joint_names = req.old_plan.joint_trajectory.joint_names
        edited_plan.joint_trajectory.points = req.old_plan.joint_trajectory.points[min_index:]
        edited_plan.joint_trajectory.points[0].positions = curr_pose

        return self.group.retime_trajectory(self.robot.get_current_state(),edited_plan,self.velocity_scaling)

if __name__ == "__main__":
    rospy.init_node('plan_reparametrizer')
    pr = plan_reparametrizer()
    s = rospy.Service('ReparametrizePlan', ReparametrizePlanPy, pr.continue_plan)
    print "Plan parametrizer ready ready"
    rospy.spin()
