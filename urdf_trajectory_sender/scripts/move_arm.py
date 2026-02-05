#!/usr/bin/env python3

import rospy
import actionlib
import argparse
import sys

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('trajectory_sender_node')

    parser = argparse.ArgumentParser(description='Send joint trajectory action goal')
    parser.add_argument('base_link_to_limb1', type=float)
    parser.add_argument('limb1_to_limb2', type=float)
    parser.add_argument('limb2_to_limb3', type=float)
    parser.add_argument('limb3_to_limb4', type=float)
    parser.add_argument('time_to_reach', type=float, help='Time to reach in seconds')

    args = parser.parse_args()

    action_name = '/urdf_1/urdf_trajectory_controller/follow_joint_trajectory'
    state_topic = '/urdf_1/urdf_trajectory_controller/state'

    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

    rospy.loginfo('Waiting for action server...')
    client.wait_for_server()
    rospy.loginfo('Action server available.')

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [
        'limb1_to_base_link',
        'limb2_to_limb1',
        'limb3_to_limb2',
        'limb4_to_limb3'
    ]

    point = JointTrajectoryPoint()
    point.positions = [
        args.base_link_to_limb1,
        args.limb1_to_limb2,
        args.limb2_to_limb3,
        args.limb3_to_limb4
    ]
    point.time_from_start = rospy.Duration(args.time_to_reach)
    goal.trajectory.points.append(point)

    rospy.loginfo('Sending goal and monitoring limb2_to_limb1...')
    client.send_goal(goal)

    # MONITORING LOOP: Watch the overshoot happen in real-time
    rate = rospy.Rate(10) # 10Hz
    while not client.wait_for_result(rospy.Duration(0.1)):
        try:
            msg = rospy.wait_for_message(state_topic, JointTrajectoryControllerState, timeout=0.1)
            idx = msg.joint_names.index('limb2_to_limb1')
            actual_pos = msg.actual.positions[idx]
            actual_vel = msg.actual.velocities[idx]
            
            # Print a single-line status update
            sys.stdout.write(f"\rMonitoring limb2_to_limb1 -> Pos: {actual_pos:.4f} rad | Vel: {actual_vel:.4f} rad/s")
            sys.stdout.flush()
        except (rospy.ROSException, ValueError):
            continue

    print("\n") # New line after the monitoring loop
    result = client.get_result()
    
    if result:
        rospy.loginfo('Goal successfully completed.')
        
        # Capture the final state after a short delay to allow physics to settle
        rospy.sleep(1.0) 
        state_msg = rospy.wait_for_message(state_topic, JointTrajectoryControllerState, timeout=2.0)
        
        print("\n--- Final Joint Status Report ---")
        for i, name in enumerate(state_msg.joint_names):
            target = point.positions[i]
            actual = state_msg.actual.positions[i]
            error = state_msg.error.positions[i]
            vel = state_msg.actual.velocities[i]
            
            # Effort (Torque) is only available if the controller is actively publishing it
            effort = state_msg.actual.effort[i] if state_msg.actual.effort else 0.0
            
            print(f"Joint: {name}")
            print(f"  Target:   {target:.4f} rad")
            print(f"  Actual:   {actual:.4f} rad")
            print(f"  Error:    {error:.4f} rad")
            print(f"  Velocity: {vel:.4f} rad/s")
            print(f"  Effort:   {effort:.4f} Nm")
            print("-" * 30)
            
        if abs(state_msg.error.positions[state_msg.joint_names.index('limb2_to_limb1')]) > 0.01:
            rospy.logwarn("High overshoot/error detected on limb2_to_limb1. Check D gain and I-clamp.")
    else:
        rospy.logerr('Goal failed.')

if __name__ == '__main__':
    main()