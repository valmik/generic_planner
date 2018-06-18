#!/usr/bin/env python

# System
import sys

# ROS
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, PlanningScene, CollisionObject, PositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from std_msgs.msg import Header



class PathPlanner(object):
  def __init__(self, group_name):
    """
    Basic barebones path planner class
    """

    rospy.loginfo("To stop project CTRL + C")
    rospy.on_shutdown(self.shutdown)


    moveit_commander.roscpp_initialize(sys.argv)

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = group_name
    self.group = moveit_commander.MoveGroupCommander(group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    DisplayTrajectory,
                                                    queue_size=20)

    # Kinematics Solvers:
    rospy.wait_for_service('/compute_fk')
    self.fk_solver = rospy.ServiceProxy('/compute_fk', GetPositionFK)

    rospy.wait_for_service('/compute_ik')
    self.ik_solver = rospy.ServiceProxy('/compute_ik', GetPositionIK)


    rospy.sleep(2)

    self.rate = rospy.Rate(10)

  def shutdown(self):
    """
    Put things to do upon shutdown
    """
    rospy.loginfo("Stopping Project")
    rospy.sleep(1)


  def plan_to_config(self, end_state):
    """
    Uses moveit to plan a path from the current joint state to the end state and returns it

    end_state: list of joint values
    """

    robot_state = self.robot.get_current_state()
    joint_state = robot_state.joint_state
    joint_state.header.stamp = rospy.Time.now()

    joint_state.position = end_state

    self.group.set_start_state_to_current_state()

    try:  
      self.group.set_joint_value_target(joint_state)
    except moveit_commander.MoveItCommanderException:
      pass

    plan = self.group.plan()

    return plan

  def execute_path(self, path, wait_bool = True):
    """
    Executes a provided path
    Note that the current position must be the same as the path's initial position
    This isn't checked

    wait_bool determines whether ROS locks the thread until the robot has completed motion
    """

    self.group.execute(path, wait = wait_bool)

  def make_pose(self, position, orientation, frame):
    """
    Creates a PoseStamped message based on the provided position and orientation
    position: list of size 3
    orientation: list of size 4 (quaternion, wxyz)
    frame: string (reference frame to which the pose is defined)
    returns pose: a PoseStamped object
    """

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.w = orientation[0]
    pose.pose.orientation.x = orientation[1]
    pose.pose.orientation.y = orientation[2]
    pose.pose.orientation.z = orientation[3]
    return pose

  def get_ik(self, pose_stamped, joint_seed = None):
    """
    Computes the inverse kinematics
    returns a list of joint angles

    if joint_seed is not specified, it will use the robot's current position
    """

    robot_state = self.robot.get_current_state()    

    if joint_seed is not None:
      robot_state.joint_state.position = joint_seed

    req = PositionIKRequest()
    req.group_name = self.group_name
    req.robot_state = self.robot.get_current_state()
    req.avoid_collisions = True
    req.ik_link_name = self.group.get_end_effector_link()
    req.pose_stamped = pose_stamped


    try:
      res = self.ik_solver(req)
      return res.solution.joint_state.position
    except rospy.ServiceException, e:
      print("IK service call failed: {}".format(e))

  def get_fk(self, joints):
    """
    Gets forward kinematics to the end effector
    joints: size 6 list. Joint angles for desired pose
    returns pose: StackedPose of the end effector in the 'root' frame
    """

    header = Header()
    header.frame_id = self.group.get_planning_frame()

    robot_state = self.robot.get_current_state()
    robot_state.joint_state.position = joints

    links = [self.group.get_end_effector_link()]

    return self.fk_solver(header, links, robot_state).pose_stamped[0]


def test_planning():
  """
  Move back and forth between two points
  """

  joints1 = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
  joints2 = [4.80, 2.92, 1.00, 4.20, 1.45, 1.32]


  path_planner = PathPlanner("manipulator")

  print path_planner.group.get_end_effector_link()

  while True:
      raw_input("Press Enter to move to position 1")
      plan = path_planner.plan_to_config(joints1)
      path_planner.execute_path(plan)
      rospy.sleep(0.5)

      raw_input("Press Enter to move to position 2")
      plan = path_planner.plan_to_config(joints2)
      path_planner.execute_path(plan)
      rospy.sleep(0.5)

def test_ik():
  """
  Tests the IK solver
  """

  path_planner = PathPlanner("manipulator")

  joints_old = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
  pose = path_planner.get_fk(joints_old)

  raw_input("Press Enter to run inverse kinematics")
  joints = path_planner.get_ik(pose)

  print 'input:' + str(joints_old)
  print 'solution:' + str(joints)

  raw_input("Press Enter to move to position")
  plan = path_planner.plan_to_config(joints)
  path_planner.execute_path(plan)
  rospy.sleep(0.5)

def test_fk():
    """
    Tests that you can get a pose from a known valid configuration
    """

    joints = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]

    path_planner = PathPlanner("manipulator")

    pose = path_planner.get_fk(joints)

    print pose


def main():
  
  rospy.init_node('path_planner', anonymous=True)

  # test_planning()
  test_fk()

  # print "Reference frame: %s" % group.get_planning_frame()
  # print "End effector: %s" % group.get_end_effector_link()
  # print "Robot groups: %s" % robot.get_group_names()






if __name__ == '__main__':
  main()


