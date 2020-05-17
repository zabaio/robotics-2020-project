import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])
plan_pos_arm = np.array([])
plan_vel_arm = np.array([])
plan_acc_arm = np.array([])
plan_pos_forearm = np.array([])
plan_vel_forearm = np.array([])
plan_acc_forearm = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_arm = np.append(plan_pos_arm, msg.positions[0])
        plan_pos_forearm = np.append(plan_pos_forearm, msg.positions[1])
        plan_vel_arm = np.append(plan_vel_arm, msg.velocities[0])
        plan_vel_forearm = np.append(plan_vel_forearm, msg.velocities[1])
        plan_acc_arm = np.append(plan_acc_arm, msg.accelerations[0])
        plan_acc_forearm = np.append(plan_acc_forearm, msg.accelerations[1])

bag.close()

# Plot data
# Planned trajectory arm
plt.figure(1)

plt.subplot(3, 1, 1)
plt.plot(plan_t,plan_pos_arm)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_t,plan_vel_arm)
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_t,plan_acc_arm)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s^2]')
plt.grid(True)

# Planned trajectory forearm
plt.figure(2)

plt.subplot(3, 1, 1)
plt.plot(plan_t,plan_pos_forearm)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (forearm)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_t,plan_vel_forearm)
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_t,plan_acc_forearm)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s^2]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
