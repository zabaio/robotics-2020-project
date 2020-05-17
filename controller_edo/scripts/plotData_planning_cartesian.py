import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

plan_t = np.array([])
plan_pos_arm = np.array([])
plan_vel_arm = np.array([])
plan_pos_forearm = np.array([])
plan_vel_forearm = np.array([])
plan_cart_t = np.array([])
plan_cart_pos = np.array([])
plan_cart_vel = np.array([])
plan_cart_acc = np.array([])

for topic, msg, t in bag.read_messages():
    if topic == "/joint_trajectory":
        plan_t = np.append(plan_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_pos_arm = np.append(plan_pos_arm, msg.positions[0])
        plan_pos_forearm = np.append(plan_pos_forearm, msg.positions[1])
        plan_vel_arm = np.append(plan_vel_arm, msg.velocities[0])
        plan_vel_forearm = np.append(plan_vel_forearm, msg.velocities[1])
    if topic == "/cartesian_trajectory":
        plan_cart_t = np.append(plan_cart_t, float(msg.time_from_start.secs+msg.time_from_start.nsecs*1.0e-9))
        plan_cart_pos = np.append(plan_cart_pos, msg.positions[0])
        plan_cart_vel = np.append(plan_cart_vel, msg.velocities[0])
        plan_cart_acc = np.append(plan_cart_acc, msg.accelerations[0])

bag.close()

# Plot data
# Planned trajectory arm
plt.figure(1)

plt.subplot(2, 1, 1)
plt.plot(plan_t,plan_pos_arm)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (arm)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(plan_t,plan_vel_arm)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

# Planned trajectory forearm
plt.figure(2)

plt.subplot(2, 1, 1)
plt.plot(plan_t,plan_pos_forearm)
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (forearm)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(plan_t,plan_vel_forearm)
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.grid(True)

# Planned Cartesian trajectory
plt.figure(3)

plt.subplot(3, 1, 1)
plt.plot(plan_cart_t,plan_cart_pos)
plt.ylabel('Position [m]')
plt.title('Planned end-effector Cartesian trajectory')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plan_cart_t,plan_cart_vel)
plt.ylabel('Velocity [m/s]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plan_cart_t,plan_cart_acc)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
