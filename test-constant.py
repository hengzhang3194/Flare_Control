import simcontrol

port = 25556
steps = 5
control_pwm = 1.0

controller = simcontrol.Controller("localhost", port)

# setup controls
control_names = ['fl', 'fr', "bl", "br"]
control_indices = []
controls = {}
for name in control_names:
    idx = controller.get_control_index(name)
    if idx is None:
        print(f'Control not found: {name}')
        continue

    control_indices.append(idx)
    controls[idx] = control_pwm

# simulation loop
total_it = 5
steps_per_it = 500
for it in range(total_it):
    print("Simulating iteration {0}...".format(it))
    controller.reset()
    for i in range(steps_per_it):
        reply = controller.simulate(steps, controls)
        if reply is None:
            print('Simulation failed, finish iteration early.')
            break
    
controller.close()