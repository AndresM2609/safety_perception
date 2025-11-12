import pickle

with open("/home/hri25-group3/ros/src/safety_perception/final_pairs_fast.pkl", "rb") as f:
    plans = pickle.load(f)
slow_factor = 1.5  # make motion 2x slower

for key, traj in plans.items():
    if not hasattr(traj, "joint_trajectory"):
        continue

    jt = traj.joint_trajectory
    new_points = []

    for pt in jt.points:
        # Scale time_from_start
        secs = pt.time_from_start.secs * slow_factor
        nsecs = pt.time_from_start.nsecs * slow_factor

        # Convert any overflow from nsecs to secs
        total_nsecs = int(secs * 1e9 + nsecs)
        new_secs = total_nsecs // int(1e9)
        new_nsecs = total_nsecs % int(1e9)

        pt.time_from_start.secs = int(new_secs)
        pt.time_from_start.nsecs = int(new_nsecs)

        # Scale velocities and accelerations too
        if hasattr(pt, "velocities"):
            pt.velocities = [v / slow_factor for v in pt.velocities]
        if hasattr(pt, "accelerations"):
            pt.accelerations = [a / (slow_factor**2) for a in pt.accelerations]

        new_points.append(pt)

    jt.points = new_points
    traj.joint_trajectory = jt
    plans[key] = traj

with open("/home/hri25-group3/ros/src/safety_perception/Timeea_test_pairs_slow.pkl", "wb") as f:
    pickle.dump(plans, f)


print("✅ Slower trajectory saved to final_pairs_slow.pkl")

