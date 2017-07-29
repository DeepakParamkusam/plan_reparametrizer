#!/usr/bin/env python

import os
import sys
import glob
import rospy
import rospkg
import moveit_commander
from yaml import load
from munch import munchify
from moveit_msgs.msg import RobotTrajectory
from rosparam import upload_params, load_file
from plan_reparametrizer.srv import ReparametrizePlanPy

def load_from_yaml(path):
    f = open(path,'r')
    yaml_file = load(f)
    f.close()
    upload_params('/loaded_yaml/',yaml_file)

def gen_plan():
    param = '/loaded_yaml/trajectory/'
    plan = RobotTrajectory()
    plan.joint_trajectory.header = munchify(rospy.get_param(param + 'header'))
    plan.joint_trajectory.joint_names = rospy.get_param(param + 'joint_names')
    plan.joint_trajectory.points = munchify(rospy.get_param(param + 'points'))
    return plan

def get_yaml_dict():
    yaml_dict = {}
    ext = '.yaml'
    pkg_func = rospkg.RosPack()
    yamls_direc = pkg_func.get_path('data') + '/trajectories'

    yaml_names = [i for i in os.listdir(yamls_direc) if os.path.splitext(i)[1] == ext]
    yaml_names = [i.strip(ext) for i in yaml_names]
    yaml_paths = glob.glob(yamls_direc + '/*.yaml')

    for i in range(len(yaml_names)):
        yaml_dict[yaml_names[i]] = yaml_paths[i]

    return yaml_dict

def continue_yaml(from_pose,to_pose):
    rospy.init_node('execute_yaml')
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander("robot")
    rospy.wait_for_service('/ReparametrizePlan')
    get_new_plan = rospy.ServiceProxy('ReparametrizePlan', ReparametrizePlanPy)


    yaml_dict = get_yaml_dict()
    req_yaml = from_pose + '_to_' + to_pose

    load_from_yaml(yaml_dict[req_yaml])
    plan = gen_plan()
    # print(type(plan.joint_trajectory))
    new_plan = get_new_plan(plan)
    # print(temp.new_traj)
    # plan.joint_trajectory = temp.new_traj
    group.execute(new_plan)

    moveit_commander.os._exit(0)

if __name__=='__main__':
  try:
      continue_yaml('pick1','place')
  except rospy.ROSInterruptException:
      pass
