import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

#Parameters
start_delay = 0.5

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


torque_t = np.array([])
torque_joint_1 = np.array([])
torque_joint_2 = np.array([])
torque_joint_3 = np.array([])

jointstate_t = np.array([])
jointstate_pos_joint_1 = np.array([])
jointstate_vel_joint_1 = np.array([])
jointstate_pos_joint_2 = np.array([])
jointstate_vel_joint_2 = np.array([])
jointstate_pos_joint_3 = np.array([])
jointstate_vel_joint_3 = np.array([])

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

    if topic == "/joint_torque":
        torque_t = np.append(torque_t, msg.data[0])
        torque_joint_1 = np.append(torque_joint_1, msg.data[1])
        torque_joint_2 = np.append(torque_joint_2, msg.data[2])
        torque_joint_3 = np.append(torque_joint_3, msg.data[3])

    if topic == "/joint_states":
        jointstate_t = np.append(jointstate_t,       float(msg.header.stamp.secs+msg.header.stamp.nsecs*1.0e-9))
        jointstate_pos_joint_1 = np.append(jointstate_pos_joint_1, msg.position[0])
        jointstate_pos_joint_2 = np.append(jointstate_pos_joint_2, msg.position[1])
        jointstate_pos_joint_3 = np.append(jointstate_pos_joint_3, msg.position[2])
        jointstate_vel_joint_1 = np.append(jointstate_vel_joint_1, msg.velocity[0])
        jointstate_vel_joint_2 = np.append(jointstate_vel_joint_2, msg.velocity[1])
        jointstate_vel_joint_3 = np.append(jointstate_vel_joint_3, msg.velocity[2])

bag.close()

# Plot data
# Planned trajectory joint_1
plt.figure(1)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_joint_1,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_joint_1,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_1)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_joint_1,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_joint_1,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Planned trajectory joint_2
plt.figure(2)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_joint_2,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_joint_2,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_2)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_joint_2,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_joint_2,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Planned trajectory joint_2
plt.figure(3)

plt.subplot(2, 1, 1)
ref, = plt.plot(plan_t,plan_pos_joint_3,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_pos_joint_3,label='actual')
plt.ylabel('Position [rad]')
plt.title('Planned trajectory (joint_3)')
plt.legend(handles=[ref, act])
plt.grid(True)

plt.subplot(2, 1, 2)
ref, = plt.plot(plan_t,plan_vel_joint_3,label='reference')
act, = plt.plot(jointstate_t-start_delay,jointstate_vel_joint_3,label='actual')
plt.ylabel('Velocity [rad/s]')
plt.legend(handles=[ref, act])
plt.grid(True)

# Joint torques
plt.figure(4)

plt.subplot(3, 1, 1)
plt.plot(torque_t,torque_joint_1)
plt.ylabel('Torque joint_1 [Nm]')
plt.title('Joint torques')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(torque_t,torque_joint_2)
plt.ylabel('Torque joint_2 [Nm]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(torque_t,torque_joint_3)
plt.ylabel('Torque joint_3 [Nm]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()
