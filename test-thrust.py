import sys
sys.path.append('../')
from simcontrol import simcontrol2
import matplotlib.pyplot as plt
from simple_pid import PID

port = 25556
test_rpms = [1000, 2000, 3000, 4000, 5000]
# test_rpms = [1000, 2000, 3000, 4000]
duration = 1.0
average_from = 0.7
control_frequency = 500.0

controller = simcontrol2.Controller("localhost", port)

rotor_names = ['fl', 'fr', 'bl', 'br']

# setup actuators
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

    actuator_name_to_index[name] = info[0]
    actuator_inputs[info[0]] = tuple([0.0])

# setup sensors
sensor_index_to_name = {}
sensor_name_to_index = {}
imu_info = controller.get_sensor_info("imu")
if imu_info is None:
    print('IMU not found!')
else:
    sensor_index_to_name[imu_info[0]] = "imu"
    
for name in rotor_names:
    info = controller.get_sensor_info("force_" + name)
    if info is None:
        print(f'Sensor not found: force_{name}')
        continue

    if (info[1] != 1):
        print(f'Sensor dof not equal to 1: force_{name}')

    sensor_name_to_index["force_" + name] = info[0]
    sensor_index_to_name[info[0]] = "force_" + name

joint_sensor_index_to_actuator_index = {}
joint_sensor_index_to_name = {}
for name in rotor_names:
    info = controller.get_sensor_info("joint_" + name)
    if info is None:
        print(f'Sensor not found: joint_{name}')
        continue

    if (info[1] != 1):
        print(f'Sensor dof not equal to 1: joint_{name}')

    sensor_name_to_index["joint_" + name] = info[0]
    sensor_index_to_name[info[0]] = "joint_" + name
    joint_sensor_index_to_name[info[0]] = name
    joint_sensor_index_to_actuator_index[info[0]] = actuator_name_to_index[name]

# simulation loop
print("Starting...")
controller.start()

time_step = controller.get_time_step()
steps_per_call = int(1.0 / control_frequency / time_step)
total_steps = duration / time_step
dt = time_step * steps_per_call

force_seq = []

for test_rpm in test_rpms:
    print(f"Testing for rpm: {test_rpm}...")

    # setup pid controller
    target_velocity = test_rpm / 30.0 * 3.14159265359
    actuator_index_to_controller = {}
    for actuator_index in actuator_inputs:
        actuator_index_to_controller[actuator_index] = PID(0.1, 0.006, 0.0, setpoint=target_velocity, sample_time=None)

    controller.reset()

    steps = 0
    t = 0.0
    
    thrust_sum = 0.0
    thrust_samples = 0
        
    while (steps < total_steps):
        reply = controller.simulate(steps_per_call, actuator_inputs)
        steps += steps_per_call
        t += dt
        if reply[0] != 0:
            print('Simulation failed, finish iteration early.')
            break
        else:
            for name in rotor_names:
                joint_sensor_idx = sensor_name_to_index["joint_" + name]
                force_sensor_idx = sensor_name_to_index["force_" + name]
                velocity = abs(reply[1][joint_sensor_idx][0])
                actuator_idx = joint_sensor_index_to_actuator_index[joint_sensor_idx]
                input = max(min(actuator_index_to_controller[actuator_idx](velocity, dt=dt), 1), 0)
                actuator_inputs[actuator_idx] = tuple([input])
                if t > average_from:
                    thrust_samples += 1
                    thrust_sum += reply[1][force_sensor_idx][0]

    avg = thrust_sum / thrust_samples
    force_seq.append(avg)
    print(f'Average thrust: {avg}')

print("Clearing...")
controller.clear()
controller.close()
print("Done.")

fig, axes = plt.subplots(1, 1)

ax1 = axes

color1 = 'tab:red'
ax1.set_xlabel('rpm')
ax1.set_ylabel('thrust (N)', color=color1)
ax1.plot(test_rpms, force_seq)
ax1.tick_params(axis='y', labelcolor=color1)
ax1.legend()

ax1.set_title('Thrust vs. RPM')

plt.show()