import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])
plan_pos_joint_1 = np.array([])
plan_vel_joint_1 = np.array([])
plan_acc_joint_1 = np.array([])
plan_pos_joint_2 = np.array([])
plan_vel_joint_2 = np.array([])
plan_acc_joint_2 = np.array([])
plan_pos_joint_3 = np.array([])
plan_vel_joint_3 = np.array([])
plan_acc_joint_3 = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_joint_1 = np.append(plan_pos_joint_1, msg.positions[0])
        plan_pos_joint_2 = np.append(plan_pos_joint_2, msg.positions[1])
        plan_pos_joint_3 = np.append(plan_pos_joint_3, msg.positions[2])
        plan_vel_joint_1 = np.append(plan_vel_joint_1, msg.velocities[0])
        plan_vel_joint_2 = np.append(plan_vel_joint_2, msg.velocities[1])
        plan_vel_joint_3 = np.append(plan_vel_joint_3, msg.velocities[2])
        plan_acc_joint_1 = np.append(plan_acc_joint_1, msg.accelerations[0])
        plan_acc_joint_2 = np.append(plan_acc_joint_2, msg.accelerations[1])
        plan_acc_joint_3 = np.append(plan_acc_joint_3, msg.accelerations[2])

bag.close()

# Plot data
# Planned trajectory joint_1
plt.figure(1)

plt.subplot(3, 1, 1)
plt.plot(plan_t,plan_pos_joint_1)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_1)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_t,plan_vel_joint_1)
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_t,plan_acc_joint_1)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s^2]')
plt.grid(True)

# Planned trajectory joint_2
plt.figure(2)

plt.subplot(3, 1, 1)
plt.plot(plan_t,plan_pos_joint_2)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_2)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_t,plan_vel_joint_2)
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_t,plan_acc_joint_2)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s^2]')
plt.grid(True)

# Planned trajectory joint_3
plt.figure(3)

plt.subplot(3, 1, 1)
plt.plot(plan_t,plan_pos_joint_3)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_3)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_t,plan_vel_joint_3)
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_t,plan_acc_joint_3)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s^2]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
