#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        group_name_eef = "gripper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group_eef = moveit_commander.MoveGroupCommander(group_name_eef)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        #print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print("============ Printing robot state")
        #print(robot.get_current_state())
        print("Printing robot pose")
        print(move_group.get_current_pose())
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.move_group_eef = move_group_eef


    def go_to_pose_goal(self,object,px,py,pz):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        #roll, pitch, yaw = -1.5707926634636131, -0.038126439723985536, 1.5708130986956612
        #quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        # pose_goal.orientation.x = quaternion[0]
        # pose_goal.orientation.y = quaternion[1]
        # pose_goal.orientation.z = quaternion[2]
        # pose_goal.orientation.w = quaternion[3]

        if object == 'return':
            pose_goal.orientation.x = -0.4888603580844928
            pose_goal.orientation.y = -0.5108936390461112
            pose_goal.orientation.z = 0.4917950669205986
            pose_goal.orientation.w = 0.5080756361293934

        elif object == 'pick1_b':
            # pick bowl/cup pose
            pose_goal.orientation.x = 0.5228742400776176
            pose_goal.orientation.y = -0.45227213971342184
            pose_goal.orientation.z = -0.5791977783366784
            pose_goal.orientation.w = 0.4319518193874791

        elif object == 'pick2_b':
            # pick bowl/cup pose
            pose_goal.orientation.x = 0.19904222232336102
            pose_goal.orientation.y = 0.14949496054346872
            pose_goal.orientation.z = -0.9276916463691346
            pose_goal.orientation.w = 0.2782474793445742

            # pose_goal.orientation.x = 0.6918673542970554
            # pose_goal.orientation.y = -0.13436795232744825
            # pose_goal.orientation.z = -0.706931813272109
            # pose_goal.orientation.w = 0.05926406018093858

        elif object == 'place1_b':
            pose_goal.orientation.x = -0.6803927797348241
            pose_goal.orientation.y = -0.1925692234419383
            pose_goal.orientation.z = 0.007155811404215404
            pose_goal.orientation.w = 0.7070583807797172

        elif object == 'place2_b':
            # pose_goal.orientation.x = -0.6644456280655098
            # pose_goal.orientation.y = -0.24194570586306285
            # pose_goal.orientation.z = -0.05497838761486289
            # pose_goal.orientation.w = 0.7049479836515066

            pose_goal.orientation.x = -0.6644456280655098
            pose_goal.orientation.y = -0.24194570586306285
            pose_goal.orientation.z = -0.05497838761486289
            pose_goal.orientation.w = 0.7049479836515066

        elif object == 'place3_b':
            pose_goal.orientation.x = -0.4173380088315606
            pose_goal.orientation.y = -0.5708284345275588
            pose_goal.orientation.z = 0.4206158152323722
            pose_goal.orientation.w = 0.568389145476699

        elif object == 'pick1_p':
            # pick up plate
            pose_goal.orientation.x = 0.35896019392733175
            pose_goal.orientation.y = -0.11560544863038387
            pose_goal.orientation.z = 0.9218725791340971
            pose_goal.orientation.w = 0.08907248319917933

        elif object == 'pick2_p':
            # pick up plate
            pose_goal.orientation.x = -0.7093077651105049
            pose_goal.orientation.y = 0.016774102269639688
            pose_goal.orientation.z = -0.7046882880492255
            pose_goal.orientation.w = 0.0039421483026973085

        elif object == 'place1_p':
            pose_goal.orientation.x = -0.011304471107518662
            pose_goal.orientation.y = 0.02797620993643868
            pose_goal.orientation.z = -0.693340880813072
            pose_goal.orientation.w = 0.7199777521589993

        elif object == 'pick3_p':
            pose_goal.orientation.x = -0.3487946783985604
            pose_goal.orientation.y = 0.556078641057902
            pose_goal.orientation.z = -0.748481356057525
            pose_goal.orientation.w = 0.09431053448226683

        elif object == 'default':
            pose_goal.orientation.x = -0.46758214167539114
            pose_goal.orientation.y = 0.5278998019211741
            pose_goal.orientation.z = -0.4801815236790802
            pose_goal.orientation.w = 0.5216458992794605



        pose_goal.position.x = px
        pose_goal.position.y = py
        pose_goal.position.z = pz

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def gripper_open(self):
        move_group_eef = self.move_group_eef

        joint_goal = move_group_eef.get_current_joint_values()
        joint_goal[0] = -0.42
        print(joint_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group_eef.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group_eef.stop()

        # For testing:
        current_joints = move_group_eef.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def gripper_close(self):
        move_group_eef = self.move_group_eef

        joint_goal = move_group_eef.get_current_joint_values()   
        joint_goal[0] = 0.63
        print(joint_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group_eef.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group_eef.stop()

        # For testing:
        current_joints = move_group_eef.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



def main():
    try:
        input(
            "============ Press `Enter` to initialise robot ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()
        # list of pick&place
        input(
            "============ Press `Enter` to begin pick place ..."
        )

        # # BOWLLLLLLLLL 
        tutorial.gripper_open()
        print('gripper opened')
        rospy.sleep(3)
        # pick bowl
        tutorial.go_to_pose_goal('pick1_b',-0.01625885061961061,0.1947477782299021,0.08048043963820806)       
        print('gripper at pick position')
        rospy.sleep(3)
        tutorial.gripper_close()
        print('gripper closed')
        rospy.sleep(3)

        tutorial.go_to_pose_goal('pick2_b',-0.04142058565971772,0.2485207827961542,0.28453144123450874)
        
        rospy.sleep(3)

        # goto bowl
        tutorial.go_to_pose_goal('place1_b',-0.20059803983820237,-0.018891269402835364,0.28379934811987306)
        rospy.sleep(6)

        tutorial.gripper_open()
        rospy.sleep(3)
        
        #tutorial.go_to_pose_goal('place2_b',-0.21335164021458994,-0.019391471377631177,0.24696721213623044)
        tutorial.go_to_pose_goal('place2_b',-0.21335164021458994,-0.019391471377631177,0.24696721213623044) 
        rospy.sleep(3)

        tutorial.go_to_pose_goal('place3_b',-0.024229356134671365,-0.20693791678704854,0.27016811095993565)
        rospy.sleep(3)
        tutorial.go_to_pose_goal('return',-0.025224781736681922,-0.03436304983685818,0.4558544729111015)





        # # PLATEEEEE

        # tutorial.go_to_pose_goal('default',0.025838613283615267,0.03940806400349911,0.46382880338904664)
        # rospy.sleep(3)
        # tutorial.gripper_open()
        # rospy.sleep(3)
        # tutorial.go_to_pose_goal('pick1_p',0.09540259452606062,0.33580868436948674,0.2016059454563548)
        # rospy.sleep(3)

        # tutorial.go_to_pose_goal('pick2_p',0.03182600552693569,0.35703080527869846,0.1431971809830999)
        # rospy.sleep(3)

        # tutorial.gripper_close()
        # rospy.sleep(3)

        # tutorial.go_to_pose_goal('pick1_p',0.09540259452606062,0.33580868436948674,0.2016059454563548)
        # rospy.sleep(3)

        # tutorial.go_to_pose_goal('pick3_p',0.09455257113677144,0.078445702978918,0.4229185968682917)
        # rospy.sleep(3)

        # tutorial.go_to_pose_goal('place1_p',-0.27743935645445184,0.06592105495987877,0.2457115860578099)
        # rospy.sleep(3)
        # tutorial.gripper_open()
        # rospy.sleep(3)
        # #tutorial.go_to_pose_goal('place2_p',,,)
        # #rospy.sleep(3)
        # tutorial.go_to_pose_goal('default',0.025838613283615267,0.03940806400349911,0.46382880338904664)
        print('GoOD JOB! :)')




    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()