#!/usr/bin/env python
import sys, pickle
import rospy, moveit_commander
from sensor_msgs.msg import JointState
import moveit_msgs.msg

with open("/home/hri25-group3/ros/src/safety_perception/joint_snapshot1.pkl", "rb") as f:
        data = pickle.load(f)

rospy.init_node('notebook')
state_pub = rospy.Publisher("notebook_state", moveit_msgs.msg.DisplayRobotState)

IN_PKL  = "joint_snapshot1.pkl"
OUT_PKL = "good_pairs.pkl"
GROUP   = "arm"
NAMED   = "ready"    # or None
VEL     = 0.5

def to_js(s):
    js = JointState()
    js.name = s["names"]
    js.position = s["q"] 
    return js

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("label_pairs_short", anonymous=True)

    state_pub = rospy.Publisher("notebook_state", DisplayRobotState, queue_size=1, latch=True)


    with open(IN_PKL, "rb") as f: data = pickle.load(f)

    move = moveit_commander.MoveGroupCommander(GROUP)
    move.set_max_velocity_scaling_factor(VEL)
    
    #for ready
    if NAMED:
        move.set_named_target(NAMED)
        move.go(wait=True)
        move.stop()

    # append to existing file if present
    try:
        with open(OUT_PKL, "rb") as f: good = pickle.load(f)
        if not isinstance(good, list): good = []
    except Exception:
        good = []

    print("Enter=save, x=ignore, q=quit\n")

    for i in range(len(data)-1):
        for j in range(i+1, len(data)):
            start = data[i]
            end = data[j]
            js1, js2 = to_js(start), to_js(end)

            # --- start -> end ---
            move.set_joint_value_target(js1)
            move.go(wait=True)
            move.stop()

            move.set_joint_value_target(js2)
            ok = move.go(wait=True)
            move.stop()

            if not ok:# planning problem
                print(f"skip {i}->{j} (plan failed)")
                continue
            #waiting for keyboard input
            ans = input(f"[{i}->{j}] Enter=save, x=ignore, q=quit > ").strip().lower()
            
            if ans == "q": 
                raise KeyboardInterrupt
            if ans != "x":
                good.append({"dir":"i_to_j","i":i,"j":j,"start":start,"end":end})
                with open(OUT_PKL,"wb") as f: pickle.dump(good,f,pickle.HIGHEST_PROTOCOL)
                print(f"saved ({i}->{j})")

            # --- end -> start ---
            move.set_joint_value_target(js1)
            ok = move.go(wait=True)
            move.stop()
            if not ok:
                print(f"return {j}->{i} failed")
                continue
            ans = input(f"[{j}->{i}] Enter=save, x=ignore, q=quit > ").strip().lower()
            if ans == "q": raise KeyboardInterrupt
            if ans != "x":
                good.append({"dir":"j_to_i","i":j,"j":i,"start":end,"end":start})
                with open(OUT_PKL,"wb") as f: pickle.dump(good,f,pickle.HIGHEST_PROTOCOL)
                print(f"saved ({j}->{i})")

    print(f"done. total saved: {len(good)}")
    moveit_commander.roscpp_shutdown()
