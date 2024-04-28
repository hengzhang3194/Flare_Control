import sys
sys.path.append('../')
import simcontrol2
import matplotlib.pyplot as plt
from simple_pid import PID
import numpy as np
import pdb

port = 25556
target_rpm = 3999       # 螺旋桨desired值，3183 转/分
fps = 30.0              # 输出信息的频率
duration = 10.0          # 仿真时长
control_frequency = 500.0           # 控制器更新频率

controller = simcontrol2.Controller("localhost", port)

rotor_names = ['fl', 'fr', 'bl', 'br']

# setup actuators, 共计4个actuators
actuator_name_to_index = {}
actuator_inputs = {}
for name in rotor_names:
    info = controller.get_actuator_info(name)
    if info is None:
        print(f'Actuator not found: {name}')
        continue

    if info[1] != 1:
        print(f'Actuator dof not equal to 1: {name}')
        continue

    actuator_name_to_index[name] = info[0]      # 获取四个actuators所对应的index
    actuator_inputs[info[0]] = tuple([0.0])     # 为四个actuators的输出赋初值

# setup sensors，共计1个sensor-imu，4个sensor-force，4个sensor-joint。
sensor_index_to_name = {}                       # 目的是为了imu的信息
imu_info = controller.get_sensor_info("imu")
if imu_info is None:
    print('IMU not found!')
else:
    sensor_index_to_name[imu_info[0]] = "imu"   # 获取1个sensor-imu的名字


force_sensor_index_to_actuator_index = {}       # 目的是为了获得关节的力
force_sensor_index_to_name = {}
for name in rotor_names:
    info = controller.get_sensor_info("force_" + name)
    if info is None:
        print(f'Sensor not found: force_{name}')
        continue

    if (info[1] != 1):
        print(f'Sensor dof not equal to 1: force_{name}')

    sensor_index_to_name[info[0]] = "force_" + name     # 获取4个sensor-force的名字
    force_sensor_index_to_name[info[0]] = name
    force_sensor_index_to_actuator_index[info[0]] = actuator_name_to_index[name]         # 分别将4个关节和4个执行器对应起来

joint_sensor_index_to_actuator_index = {}       # 目的是为了获得关节的转速
joint_sensor_index_to_name = {}
for name in rotor_names:
    info = controller.get_sensor_info("joint_" + name)
    if info is None:
        print(f'Sensor not found: joint_{name}')
        continue

    if (info[1] != 1):
        print(f'Sensor dof not equal to 1: joint_{name}')

    sensor_index_to_name[info[0]] = "joint_" + name     # 获取4个sensor-joint的名字
    joint_sensor_index_to_name[info[0]] = name
    joint_sensor_index_to_actuator_index[info[0]] = actuator_name_to_index[name]         # 分别将4个关节和4个执行器对应起来

# setup pid controller
target_velocity = target_rpm / 30.0 * 3.14159265359     # 将desired转速从rpm变为 rad/s，即 rpm * 2 * pi / 60；
actuator_index_to_controller = {}
for actuator_index in actuator_inputs:
    actuator_index_to_controller[actuator_index] = PID(0.1, 0.006, 0.0, setpoint=target_velocity, sample_time=None)     # PID(0.1, 0.006, 0.0, setpoint=target_velocity) 
    # 每个index对应的是PID的计算程序，使用的时候需要输入当前值，示例：actuator_index_to_controller[0](current_velocity)。

# simulation loop
print("Starting...")
controller.start()
print("Resetting...")
controller.reset()
print("Exporting 0th...")
controller.export(0)

# setup mapping between targets and controls
front_len = 1
behind_len = 1
controls_to_targets = np.array(((front_len, front_len, behind_len, behind_len), (front_len, -1 * front_len, behind_len, -1 * behind_len), (front_len, front_len, -1 * behind_len, -1 * behind_len), (front_len, -1 * front_len, -1 * behind_len, behind_len))) #pitch is not balanced due to propeller position difference between front and back
targets_to_control = np.linalg.inv(controls_to_targets)

Kp = (2, 2, 20, 20, 20, 20)     # x, y, z, roll, pitch, yaw
Kd = np.dot(Kp, 10)

def get_desired(trajectory_flag, t):
    if trajectory_flag == 1:
        if t < 10:
            desired = (0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        elif t < 20:
            desired = (0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            desired = (2, -2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    elif trajectory_flag == 2:
        xd = 1 * np.exp(t / 20) * np.sin(t) + 0.2
        yd = np.exp(t / 20) * np.cos(t) - 0.824
        zd = 0.5 * t
        d_xd = 1 / 20 * np.exp(t / 20) * np.sin(t) + 1 * np.exp(t / 20) * np.cos(t)
        d_yd = 0.05 * np.exp(t / 20) * np.cos(t) - np.exp(t / 20) * np.sin(t)
        d_zd = 0.5
        desired = (xd, yd, zd, d_xd, d_yd, d_zd, 0, 0, 0, 0, 0, 0)
    return desired

time_step = controller.get_time_step()      # 获取仿真器的更新步长，0.0002 s 
steps_per_call = int(1.0 / control_frequency / time_step)       # 获取 每次控制器更新时仿真器更新的步数，0.002 / 0.0002 = 10步
total_steps = duration / time_step      # 获取总的仿真步数
export_interval = 1.0 / fps             # 获取 终端打印输出的间隔
dt = time_step * steps_per_call         # 控制器更新的时间步长，其实就是控制器的更新频率的倒数。
# pdb.set_trace()


steps = 0
export_count = 1
t = 0.0
print("Simulating...")
seq_time = [0.0]
seq_data = {}
states_all = []  # 存储所有的状态序列，12维状态
desired_states_all = []
actuator = {}
info = controller.get_actuator_info("controller")  
actuator[info[0]] = tuple([0, 0, 2, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, 0, np.nan])

for i in range(0, 12):
    states_all.append([0.0])  # 用来存储状态信息以便画图
for i in range(0, 12):
    desired_states_all.append([0.0])  # 用来存储状态信息以便画图

for name in rotor_names:                # 也就是说 seq_data 有12个元素，初值被设为0.
    seq_data["force_" + name] = [0.0]
    seq_data["joint_" + name] = [0.0]
    seq_data["pwm_" + name] = [0.0]
    
while (steps < total_steps):
    #print("Simulate steps!")
    reply = controller.simulate(steps_per_call, actuator)        # 返回的reply=(status_code, sensor_outputs)，sensor_outputs的维度等于（sensor的数量 * 相对应的自由度）。
    steps += steps_per_call         # 每次更新一个控制步。
    t += dt                         # 更新当前的时间
    if reply[0] != 0:               # reply[0] = status_code; 示例：reply[0] = 0；
        print('Simulation failed, finish iteration early.')
        break
    else:
        seq_time.append(t)          # 记录当前的时间

        states = reply[1][0]
        print(f'position: ({states[0]:.5f}, {states[1]:.5f}, {states[2]:.5f}), velocity: ({states[3]:.5f}, {states[4]:.5f}, {states[5]:.5f}), rotation: ({states[6]:.5f}, {states[7]:.5f}, {states[8]:.5f}), angular velocity: ({states[9]:.5f}, {states[10]:.5f}, {states[11]:.5f})')

        for i in range(0, 12):
            states_all[i].append(states[i])
        
        trajectory_flag = 1    # 选择desired trajectory的类型，=1 是step类型，=2 是螺旋上升类型
        desired_states = get_desired(trajectory_flag, t)
        for i in range(0, 12):
            desired_states_all[i].append(desired_states[i])

        target_output = [0, 0, 0, 0] # 返回一个和target_input同维度的全零数组

        temp_x = Kp[0] * (desired_states[0] - states[0]) + Kd[0] * (desired_states[6] - states[6])
        temp_y = Kp[1] * (desired_states[1] - states[1]) + Kd[1] * (desired_states[7] - states[7])
        target_output[0] = Kp[2] * (desired_states[2] - states[2]) + Kd[2] * (desired_states[8] - states[8])
        target_output[1] = Kp[3] * (desired_states[4] - states[4] + temp_x) + Kd[3] * (desired_states[10] - states[10])
        target_output[2] = Kp[4] * (desired_states[3] - states[3] - temp_y) + Kd[4] * (desired_states[9] - states[9])
        target_output[3] = Kp[5] * (desired_states[5] - states[5]) + Kd[5] * (desired_states[11] - states[11])

        true_control = np.dot(targets_to_control, target_output)

        # pdb.set_trace()

        # actuator = [0, 0, 2, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, 0, np.nan]

        # actuator_index_to_controller = {}
        # for actuator_index in actuator_inputs:
        #     actuator_index_to_controller[actuator_index] = PID(0.1, 0.006, 0.0, setpoint=true_control[actuator_index], sample_time=None)     # PID(0.1, 0.006, 0.0, setpoint=target_velocity) 
            # 每个index对应的是PID的计算程序，使用的时候需要输入当前值，示例：actuator_index_to_controller[0](current_velocity)。

        # pdb.set_trace()

        for idx in reply[1]:        # reply[1] = sensor_outputs; 是个tuple，示例：reply[1] = {0: (0.002,), 1: (0.002,), 2: (-0.002,), 3: (-0.002,), 4: (0.010,), 5: (-0.010,), 6: (0.009,), 7: (-0.009,)}；
            # pdb.set_trace()
            if idx in joint_sensor_index_to_actuator_index:     # 在没有imu的情况下，sensor_index_to_name = {2: 'force_fl', 3: 'force_fr', 0: 'force_bl', 1: 'force_br', 6: 'joint_fl', 7: 'joint_fr', 4: 'joint_bl', 5: 'joint_br'}；
                actuator_idx = joint_sensor_index_to_actuator_index[idx]    # 4个sensor-joint对应4个actuator
                velocity = abs(reply[1][idx][0])      # 速度都是大于0的，但是由于旋翼的正反转，所以取绝对值。
                # Assuming length is 1
                input = max(min(actuator_index_to_controller[actuator_idx](velocity, dt=dt), 1), 0)     # 根据当前的速度，计算PID的输出

                # actuator_idx = joint_sensor_index_to_actuator_index[idx]    # 4个sensor-joint对应4个actuator
                # velocity = abs(reply[1][idx][0])      # 速度都是大于0的，但是由于旋翼的正反转，所以取绝对值。
                # # Assuming length is 1
                # input = max(min(true_control[actuator_idx], 1), 0)     # 根据当前的速度，计算PID的输出


                # actuator_inputs[actuator_idx] = tuple([input])  # 这是一个包含了4个tuple的变量，每个控制步会更新
                seq_data["pwm_" + joint_sensor_index_to_name[idx]].append(0.0)    # 记录每个关节在每个时刻的PID的输出(0, 1).

                # if not isinstance(input, int):
                #     pdb.set_trace()

            if idx in sensor_index_to_name and sensor_index_to_name[idx] in seq_data:    # 上面的seq_data只更新了“pwm_”4维，本函数更新剩下的“force_”和“joint_”4+4维。
                # Assuming length is 1
                seq_data[sensor_index_to_name[idx]].append(reply[1][idx][0])
                # print(f'{sensor_index_to_name[idx]}: {reply[1][idx]}')
    
    # print('====================================================')
    
    if steps * time_step >= export_count * export_interval:      # 每秒30fps，打印信息，
        print(f"Exporting {export_count}th...")
        controller.export(export_count)
        export_count += 1

#print("Clearing...")
#controller.clear()
controller.close()
print("Done.")

for name in rotor_names:        # 将 seq_data中的“joint_”记录值从 rad/s 转化为 rpm。
    for i in range(len(seq_data["joint_" + name])):
        seq_data["joint_" + name][i] = abs(seq_data["joint_" + name][i]) * 30 / 3.14159265359



# pdb.set_trace()

# Figure 1
fig, axes = plt.subplots(1, 3)

ax1 = axes[0]
ax2 = axes[1]
ax3 = axes[2]

color1 = 'tab:red'
ax1.set_xlabel('time(s)')
ax1.set_ylabel('propeller RPM', color=color1)
ax1.plot(seq_time, seq_data["joint_fl"], label='fl')
ax1.plot(seq_time, seq_data["joint_fr"], label='fr')
ax1.plot(seq_time, seq_data["joint_bl"], label='bl')
ax1.plot(seq_time, seq_data["joint_br"], label='br')
ax1.tick_params(axis='y', labelcolor=color1)
#ax1.set_ylim(0, 10000)
ax1.legend()

ax1.set_title('Propeller RPM vs. Time')

color1 = 'tab:red'
ax2.set_xlabel('time(s)')
ax2.set_ylabel('Motor PWM', color=color1)
ax2.plot(seq_time, seq_data["pwm_fl"], label='fl')
ax2.plot(seq_time, seq_data["pwm_fr"], label='fr')
ax2.plot(seq_time, seq_data["pwm_bl"], label='bl')
ax2.plot(seq_time, seq_data["pwm_br"], label='br')
ax2.tick_params(axis='y', labelcolor=color1)
#ax2.set_ylim(0, 1)
ax2.legend()

ax2.set_title('Motor PWM vs. Time')

color1 = 'tab:red'
ax3.set_xlabel('time(s)')
ax3.set_ylabel('thrust(N)', color=color1)
ax3.plot(seq_time, seq_data["force_fl"], label='fl')
ax3.plot(seq_time, seq_data["force_fr"], label='fr')
ax3.plot(seq_time, seq_data["force_bl"], label='bl')
ax3.plot(seq_time, seq_data["force_br"], label='br')
ax3.tick_params(axis='y', labelcolor=color1)
#ax3.set_ylim(0, 10)
ax3.legend()

ax3.set_title('Thrust vs. Time')

fig.tight_layout()
fig.set_size_inches(15.0, 4.0)


# Figure 2
fig2, axes = plt.subplots(1, 4)

ax1 = axes[0]
ax2 = axes[1]
ax3 = axes[2]
ax4 = axes[3]

color1 = 'tab:red'
ax1.set_xlabel('time(s)')
ax1.set_ylabel('Position(m)', color=color1)
ax1.plot(seq_time, states_all[0], label='Px')
ax1.plot(seq_time, states_all[1], label='Py')
ax1.plot(seq_time, states_all[2], label='Pz')
ax1.tick_params(axis='y', labelcolor=color1)
#ax1.set_ylim(0, 10000)
ax1.legend()

ax1.set_title('Time vs. Position')

color1 = 'tab:red'
ax2.set_xlabel('time(s)')
ax2.set_ylabel('Angular(rad)', color=color1)
ax2.plot(seq_time, states_all[3], label='Rx')
ax2.plot(seq_time, states_all[4], label='Ry')
ax2.plot(seq_time, states_all[5], label='Rz')
ax2.tick_params(axis='y', labelcolor=color1)
ax2.legend()

ax1.set_title('Time vs. Angular')

color1 = 'tab:red'
ax3.set_xlabel('time(s)')
ax3.set_ylabel('Velocity(m)', color=color1)
ax3.plot(seq_time, states_all[6], label='Vx')
ax3.plot(seq_time, states_all[7], label='Vy')
ax3.plot(seq_time, states_all[8], label='Vz')
ax3.tick_params(axis='y', labelcolor=color1)
ax3.legend()

ax1.set_title('Time vs. Velocity')

color1 = 'tab:red'
ax4.set_xlabel('time(s)')
ax4.set_ylabel('Angular Velocity(rad/s)', color=color1)
ax4.plot(seq_time, states_all[9], label='Wx')
ax4.plot(seq_time, states_all[10], label='Wy')
ax4.plot(seq_time, states_all[11], label='Wz')
ax4.tick_params(axis='y', labelcolor=color1)
ax4.legend()

ax4.set_title('Time vs. Angular Velocity')

fig2.tight_layout()
fig2.set_size_inches(20.0, 4.0)














plt.show()









