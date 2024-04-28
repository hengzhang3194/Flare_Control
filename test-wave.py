import sys
sys.path.append('../')
from simcontrol import simcontrol2
from simple_pid import PID

port = 25556
target_rpm = 5000
fps = 30.0
duration = 3.0
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
for name in rotor_names:
    info = controller.get_sensor_info("force_" + name)
    if info is None:
        print(f'Sensor not found: force_{name}')
        continue

    if (info[1] != 1):
        print(f'Sensor dof not equal to 1: force_{name}')

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

    sensor_index_to_name[info[0]] = "joint_" + name
    joint_sensor_index_to_name[info[0]] = name
    joint_sensor_index_to_actuator_index[info[0]] = actuator_name_to_index[name]

# setup pid controller
target_velocity = target_rpm / 30.0 * 3.14159265359
actuator_index_to_controller = {}
for actuator_index in actuator_inputs:
    actuator_index_to_controller[actuator_index] = PID(0.008, 0.0002, 0.0, setpoint=target_velocity)

# Start sim
print("Starting...")
controller.start()
print("Resetting...")
controller.reset()

# simulation loop
time_step = controller.get_time_step()
steps_per_call = int(1.0 / control_frequency / time_step)
total_steps = duration / time_step
export_interval = 1.0 / fps
dt = time_step * steps_per_call

# export 0th
print("Exporting 0th...")
controller.export(0)

steps = 0
export_count = 1
print("Simulating...")
    
while (steps < total_steps):
    #print("Simulate steps!")
    reply = controller.simulate(steps_per_call, actuator_inputs)
    steps += steps_per_call
    if reply[0] != 0:
        print('Simulation failed, finish iteration early.')
        break
    else:
        for idx in reply[1]:
            if idx in joint_sensor_index_to_actuator_index:
                actuator_idx = joint_sensor_index_to_actuator_index[idx]
                velocity = abs(reply[1][idx][0])
                # Assuming length is 1
                input = max(min(actuator_index_to_controller[actuator_idx](velocity), 1), 0)
                actuator_inputs[actuator_idx] = tuple([input])
    
    # print('====================================================')
    
    if steps * time_step >= export_count * export_interval:
        print(f"Exporting {export_count}th...")
        controller.export(export_count)
        export_count += 1

print("Clearing...")
controller.clear()
controller.close()
print("Done.")