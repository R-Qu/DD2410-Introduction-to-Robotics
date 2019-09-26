#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from IPython import embed

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

previous_state = None

class StateMachine(object):

    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose = aruco_pose_msg
        self.aruco_pose_rcv = True
    def gripper_cb(self, joint_state_msg):
        self.left_gripper = joint_state_msg.position[7]
        self.right_gripper = joint_state_msg.position[8]

    def __init__(self):

        self.node_name = "Student SM"

        self.aruco_pose = None
        self.aruco_pose_rcv = False
        self.left_gripper = None
        self.right_gripper = None

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pk_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.plc_srv = rospy.get_param(rospy.get_name() + '/place_srv')
        self.dtct_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.lclz_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.clrcstmp_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.pkpose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.plcpose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # Subscribe to topics
        self.dtct_sub = rospy.Subscriber(self.dtct_top, PoseStamped, self.aruco_pose_cb)
        self.gripper_sub = rospy.Subscriber("/joint_states", JointState, self.gripper_cb)

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pk_srv, timeout=30)
        rospy.wait_for_service(self.plc_srv, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        rospy.loginfo("%s: Waiting for move base goal server...", self.node_name)
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base action server", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()

    def check_states(self):

        while not rospy.is_shutdown() and self.state != 50:

            # State 0:  Tuck arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                fail_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(10.0))

                if fail_tucking:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    previous_state = 0
                    self.state = 0
                else:
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                    previous_state = 0
                    self.state = 1

                rospy.sleep(1)

            # State 1: Localize robot
            if self.state == 1:
                move_msg = Twist()
                move_msg.angular.z = -1
                rospy.loginfo("%s: Localizing Rob position...", self.node_name)
                localize_srv = rospy.ServiceProxy(self.lclz_srv, Empty)
                localize_req = localize_srv()

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Doing a Barrel Roll...", self.node_name)
                while not rospy.is_shutdown() and cnt < 60:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                clear_costmap_srv = rospy.ServiceProxy(self.clrcstmp_srv, Empty)
                clear_costamp_req = clear_costmap_srv()

                previous_state = 1
                self.state = 2

                rospy.sleep(1)

            # State 2:  Navigate to the cube
            if self.state == 2:
                rospy.loginfo("%s: Navigating to cube...", self.node_name)
                pose = rospy.wait_for_message(self.pkpose_top, PoseStamped, 5)
                goal = MoveBaseGoal()

                goal.target_pose = pose
                self.move_base_ac.send_goal(goal)
                success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(30.0))

                if not success_navigation:
                    self.move_base_ac.cancel_goal()
                    rospy.logerr("%s: Failed to navigate to cube, reset simulation", self.node_name)
                    previous_state = 2
                    self.state = 3
                else:
                    rospy.loginfo("%s: Cube reached!", self.node_name)
                    previous_state = 2
                    self.state = 3

                rospy.sleep(1)

            # State 3:  Lower robot head service
            if self.state == 3:
            	try:
                    rospy.loginfo("%s: Lowering robot head...", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")

                    if move_head_req.success == True:
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                        previous_state = 3
                        self.state = 7

                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        previous_state = 3
                        self.state = 404

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # State 4:  Pick up the cube service
            if self.state == 4:
            	try:
                    rospy.loginfo("%s: Picking the cube...", self.node_name)
                    pick_up_srv = rospy.ServiceProxy(self.pk_srv, SetBool)
                    pick_up_req = pick_up_srv()

                    if pick_up_req.success == True:
                        previous_state = 4
                        self.state = 7
                    else:
                        rospy.loginfo("%s: Pick up failed!", self.node_name)
                        previous_state = 4
                        self.state = 404

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to pick_up server failed: %s"%e

            # State 5:  Navigate to table A
            if self.state == 5:
                rospy.loginfo("%s: Navigating to table...", self.node_name)
                pose = rospy.wait_for_message(self.plcpose_top, PoseStamped, 5)
                goal = MoveBaseGoal()

                goal.target_pose = pose
                self.move_base_ac.send_goal(goal)
                success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(120.0))

                if not success_navigation:
                    self.move_base_ac.cancel_goal()
                    rospy.logerr("%s: Failed to navigate to table, reset simulation", self.node_name)
                    previous_state = 5
                    self.state = 404
                else:
                    rospy.loginfo("%s: Table reached!", self.node_name)
                    previous_state = 5
                    self.state = 6

                rospy.sleep(1)


            # State 6:  Place the cube service
            if self.state == 6:
                try:
                    rospy.loginfo("%s: Placing the cube...", self.node_name)
                    place_srv = rospy.ServiceProxy(self.plc_srv, SetBool)
                    place_req = place_srv()

                    if place_req.success == True:
                        previous_state = 6
                        self.state = 7
                        rospy.loginfo("%s: Placing succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Placing failed!", self.node_name)
                        previous_state = 6
                        self.state = 404

                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    print "Service call to pick_up server failed: %s"%e

            # State 7: Detect the cube
            if self.state == 7:

                if previous_state == 3:
                    move_msg = Twist()
                    move_msg.angular.z = 1

                    rate = rospy.Rate(10)
                    converged = False
                    cnt = 0
                    self.aruco_pose_rcv = False
                    rospy.loginfo("%s: Checking if cube is in sight...", self.node_name)
                    while not rospy.is_shutdown() and self.aruco_pose_rcv == False and cnt < 75:
                        self.cmd_vel_pub.publish(move_msg)
                        cnt += 1
                        rate.sleep()

                    previous_state = 7
                    self.state = 4
                    rospy.sleep(1)

                if previous_state == 4:
                    if self.left_gripper > 0.02 and self.right_gripper > 0.02:
                        rospy.loginfo("%s: Pick up succeded!", self.node_name)
                        move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                        move_head_req = move_head_srv("up")
                        clear_costmap_srv = rospy.ServiceProxy(self.clrcstmp_srv, Empty)
                        clear_costamp_req = clear_costmap_srv()
                        previous_state = 7
                        self.state = 5
                    else:
                        rospy.loginfo("%s: ATTENTION: CUBE DROPPED!", self.node_name)
                        previous_state = 7
                        self.state = 404

                if previous_state == 6:
                    try:
                        rospy.loginfo("%s: Detecting the placed cube...", self.node_name)
                        move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                        move_head_req = move_head_srv("down")
                        self.aruco_pose_rcv = False
                        cnt = 0
                        while not rospy.is_shutdown() and self.aruco_pose_rcv == False and cnt < 5:
                            cnt += 1
                            rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection...")
                            rospy.sleep(1.0)

                        if cnt < 5 :
                            rospy.loginfo("%s: CUBE DETECTED!", self.node_name)
                            previous_state = 7
                            self.state = 50
                        else :
                            rospy.loginfo("%s: Cannot find the cube...", self.node_name)
                            previous_state = 7
                            self.state = 404

                        rospy.sleep(3)

                    except rospy.ServiceException, e:
                        print "Service call to pick_up server failed: %s"%e

            # Error handling

            if self.state == 404:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
