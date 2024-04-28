import sys
sys.path.append('../')
from simcontrol import simcontrol2
import matplotlib.pyplot as plt
from simple_pid import PID
import time

port = 25556
#test_rpms = [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500]
test_rpms = [5000]
duration = 1.5
average_from = 1.0
control_frequency = 500.0

controller = simcontrol2.Controller("localhost", port)

# setup actuators
motor_name = 'motor'
motor_info = controller.get_actuator_info(motor_name)
if motor_info is None:
    print("motor not found.")
    exit(1)
motor_index = motor_info[0]
motor_input = {motor_index: tuple([0.0])}

# setup sensors
joint_sensor_name = 'joint'
force_sensor_name = 'force'
joint_sensor_info = controller.get_sensor_info(joint_sensor_name)
force_sensor_info = controller.get_sensor_info(force_sensor_name)
if joint_sensor_info is None:
    print("Joint sensor not found.")
    exit(1)

if force_sensor_info is None:
    print("Force sensor not found.")
    exit(1)

joint_sensor_index = joint_sensor_info[0]
force_sensor_index = force_sensor_info[0]

# simulation loop
print("Starting...")
controller.start()

time_step = controller.get_time_step()
steps_per_call = int(1.0 / control_frequency / time_step)
total_steps = duration / time_step
dt = time_step * steps_per_call

force_seq = []
pwm_seq = []
rpm_seq = []

fps = 30.0
interval = 1.0 / fps
timer = 0.0
seq = 0
last_export_time = time.time()
for test_rpm in test_rpms:
    print(f"Testing for rpm: {test_rpm}...")

    # setup pid controller
    target_velocity = test_rpm / 30.0 * 3.14159265359
    pid = PID(0.1, 0.006, 0.0, setpoint=target_velocity, sample_time=None)

    controller.reset()

    steps = 0
    t = 0.0
    
    thrust_sum = 0.0
    pwm_sum = 0.0
    rpm_sum = 0.0
    samples = 0

    test_rpm_time_seq = [0.0]
    test_rpm_pwm_seq = [0.0]
    test_rpm_rpm_seq = [0.0]
    test_rpm_thrust_seq = [0.0]
        
    while (steps < total_steps):
        timer -= time_step * steps_per_call
        if timer <= 0.0:
            this_time = time.time()
            if seq >= 1:
                print(f'Exporting {seq}th, percentage of realtime: {interval / (this_time - last_export_time) * 100:.2f}%')
            else:
                print('Exporting 0th.')
            last_export_time = this_time
            controller.export(seq)
            seq += 1
            timer += interval
        reply = controller.simulate(steps_per_call, motor_input)
        steps += steps_per_call
        t += dt
        if reply[0] != 0:
            print('Simulation failed, finish iteration early.')
            break
        else:
            velocity = abs(reply[1][joint_sensor_index][0])
            thrust = reply[1][force_sensor_index][0]
            rpm = velocity * 30.0 / 3.14159265359
            input = max(min(pid(velocity, dt=dt), 1), 0)
            motor_input[motor_index] = tuple([input])
            test_rpm_time_seq.append(t)
            test_rpm_pwm_seq.append(input)
            test_rpm_rpm_seq.append(rpm)
            test_rpm_thrust_seq.append(thrust)
            if t > average_from:
                samples += 1
                rpm_sum += rpm
                pwm_sum += input
                thrust_sum += thrust

    avg_rpm = rpm_sum / samples
    avg_pwm = pwm_sum / samples
    avg_thrust = thrust_sum / samples
    rpm_seq.append(avg_rpm)
    pwm_seq.append(avg_pwm)
    force_seq.append(avg_thrust)
    print(f'Average pwm: {avg_pwm}, thrust: {avg_thrust}, rpm: {avg_rpm}')

    fig, axes = plt.subplots(1, 3)
    ax1 = axes[0]
    ax1.set_xlabel('time(s)')
    ax1.set_ylabel('rpm')
    ax1.plot(test_rpm_time_seq, test_rpm_rpm_seq)

    ax2 = axes[1]
    ax2.set_xlabel('time(s)')
    ax2.set_ylabel('thrust(N)')
    ax2.plot(test_rpm_time_seq, test_rpm_thrust_seq)

    ax3 = axes[2]
    ax3.set_xlabel('time(s)')
    ax3.set_ylabel('pwm')
    ax3.plot(test_rpm_time_seq, test_rpm_pwm_seq)

    fig.set_size_inches(12.0, 4.0)
    plt.show()



print("Clearing...")
controller.clear()
controller.close()
print("Done.")

fig, axes = plt.subplots(1, 1)

ax1 = axes

color1 = 'tab:red'
ax1.set_xlabel('rpm')
ax1.set_ylabel('thrust (N)', color=color1)
ax1.plot(test_rpms, force_seq, color=color1)
ax1.tick_params(axis='y', labelcolor=color1)
ax1.set_xlim(0.0, 6000.0)
ax1.set_ylim(0.0, 20.0)

ax2 = ax1.twinx()
color2 = 'tab:brown'
ax2.set_ylabel('PWM', color=color2)
ax2.plot(test_rpms, pwm_seq, color=color2)
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_ylim(0.0, 1.0)

ax1.set_title('Thrust and PWM vs. RPM')

plt.show()