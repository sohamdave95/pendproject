import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import numpy as np


model = mujoco.MjModel.from_xml_path("furatamjcf.xml")
data = mujoco.MjData(model)
target_angle = 180

past_time, past_base, past_pend = [], [], []


data.joint("pend").qpos[0] = np.radians(165)

kp = 18 #25
ki = 0
kd = 0.8 #5
bias = 0

last_time = 0
last_error = 0


def pidpend():


    global last_time, last_error, current_velocity
    current_time = data.time
    current_angle = np.degrees(data.joint("pend").qpos[0]) % 360
    current_velocity = abs(data.joint("arm").qvel[0])

    delta_time = current_time - last_time


    error = (target_angle - current_angle)


    if delta_time > 1:
        derivative = (error - last_error)/delta_time
    else:
        derivative = 0

    output = (error*kp) + (derivative*kd)


    data.actuator("n20").ctrl[0] = np.clip(-output, -1, 1)


    last_time = current_time
    last_error = error

    past_pend.append(current_angle)
    past_time.append(current_time)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        pidpend()
        mujoco.mj_step(model, data)
        viewer.sync()
        print(current_velocity)



    plt.figure(figsize=(10, 5))
    #plt.plot(past_time, past_base, label='pendulum angle (deg)', color='red')
    plt.plot(past_time, past_pend, label="base angle (deg)", color="blue")
    plt.axhline(y=180, label='Baseline', color='blue', linestyle='--')

    plt.title('Furata Pendulum')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.legend()
    plt.grid(True)
    plt.show()

