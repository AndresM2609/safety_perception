#!/usr/bin/env python
import sys, pickle
import rospy, moveit_commander
from sensor_msgs.msg import JointState
import moveit_msgs.msg

with open("/home/hri25-group3/ros/src/safety_perception/joint_snapshot1.pkl", "rb") as f:
        data = pickle.load(f)

IN_PKL  = "joint_snapshot1.pkl"
OUT_PKL = "test_pairs_fast.pkl"
GROUP   = "arm"
NAMED   = "ready"    # or None
VEL_SCALING_FACTOR = 0.8  # fraction of limits #for slow 0.6
MAX_CARTESIAN_SPEED = 0.7#m/s #for slow 0.3
#for slow motions

def to_js(s):
    js = JointState()
    js.name = s["names"]
    js.position = s["q"] 
    return js

def main():
    rospy.init_node("label_pairs_short", anonymous=True)

    state_pub = rospy.Publisher("notebook_state", moveit_msgs.msg.DisplayRobotState, queue_size=1, latch=True)


    with open(IN_PKL, "rb") as f: data = pickle.load(f)

    move = moveit_commander.MoveGroupCommander(GROUP)
    move.set_max_velocity_scaling_factor(VEL_SCALING_FACTOR)
    move.limit_max_cartesian_link_speed(MAX_CARTESIAN_SPEED,'arm_tool0')
    
    #for ready
    if NAMED:
        move.set_named_target(NAMED)
        while not move.go(wait=True):
            rospy.logerr("Robot did not arrive to READY position")
            if not rospy.is_shutdown():
                return 
            rospy.sleep(3.0)



    # append to existing file if present
    try:
        with open(OUT_PKL, "rb") as f: good = pickle.load(f)
        if not isinstance(good,dict):
            raise
    except Exception:
        rospy.logwarn("could not load dictionary start from scratch")
        good = dict()

    print("Enter=save, x=ignore, q=quit\n")

    for i in range(len(data)-1):
        start = data[i]
        js1=to_js(start)
        try:
            move.set_joint_value_target(js1)
            if not move.go(wait=True):
                raise
        except Exception:
            rospy.logerr(f"Cannot reach pose {i} skipping starting point")
            continue

        for j in range(i+1, len(data)):
            if (i,j) in good:
                rospy.loginfo(f"skipping known trajectory from {i} to {j}")
                continue
            end = data[j]
            js2 = to_js(end)
            rospy.loginfo(f"{js2}")
            try:
                move.set_joint_value_target(js2)
            except Exception:
                rospy.logwarn("js2 failed")

            ok,plan,planning_time,error_code = move.plan()
       
            if not ok:# planning problem
                print(f"skip {i}->{j} (second position not reachable, failed)")
                continue
            move.execute(plan, wait=True)
            #waiting for keyboard input
            ans = input(f"[{i}->{j}] Enter=save, x=ignore, q=quit > ").strip().lower()
            
            if ans == "q": 
                return
            if ans != "x":
                good[(i,j)]= plan
                with open(OUT_PKL,"wb") as f: pickle.dump(good,f,pickle.HIGHEST_PROTOCOL)
                print(f"saved ({i}->{j})")  

            # --- end -> start ---
            move.set_start_state_to_current_state()
            move.set_joint_value_target(js1)
            ok_rev, plan_rev, _, _ = move.plan()

            if ok_rev:
                # Execute reverse and save it
                move.execute(plan_rev, wait=True)
                good[(j, i)] = plan_rev
                with open(OUT_PKL, "wb") as f:
                    pickle.dump(good, f, pickle.HIGHEST_PROTOCOL)
                print(f"saved reverse ({j}->{i})")
            else:
                rospy.logwarn(f"reverse plan {j}->{i} failed; returning with simple go()")
                move.set_joint_value_target(js1)
                move.go(wait=True)

        else:
            # Not saving forward: just go back without saving reverse
            move.set_joint_value_target(js1)
            move.go(wait=True)


            while not move.go(wait=True):
                rospy.logerr("Robot did not arrive to READY position")
                if not rospy.is_shutdown():
                    return 
                rospy.sleep(3.0)

    print(f"done. total saved: {len(good)}")


if __name__ =="__main__":
    main()

