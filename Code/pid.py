#woohoo another soham project
#python file to mujoco sim - sample pid testing
#uses class to simplify down the math - and to save in the memory which is needed since we use prior error value in memory each time


import mujoco 
import mujoco.viewer
import matplotlib.pyplot as plt
import time
import numpy 

model = mujoco.MjModel.from_xml_path("furatamjcf.xml")
data = mujoco.MjData(model)
targetangle = 180
timestep = 0.0005


#arrays for mlt plots
past_time = []
past_angle = []
target_line = [180]


# PID values
kp = 0
ki = 0
kd = 0



class pid_loop:

    def setup(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0


    def pidcalcs(self, currentangle, dt):
        #first thing we do which is read the angle
        error = currentangle - targetangle
        while error > 180: 
            error -= 360 #stops the unrestrained joint from still adding values nonstop basically
        while error < -180: 
            error += 360

        #tracking derivative and integral terms over time
        self.integral += error*dt
        deriv = (error - self.last_error)/dt

        output = (self.kp*error) + (self.ki*error) + (self.kd*error)
        self.last_error = error

        return numpy.clip(output, -1, 1)


pid = pid_loop(kp,ki,kd)


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        current_deg = numpy.degrees(data.joint("pend").qpos[0])
        signal = pid.compute(current_deg, timestep)

        data.actuator("n20").ctrl[0] = signal

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start
        if elapsed < timestep:
            time.sleep(timestep - elapsed)

        #for matplotlib visualizing - append prior data to the error for graph
        current_time = data.time
        past_time.append(current_time)
        past_angle.append(current_deg) 

    plt.figure(figsize=(10, 5))
    plt.plot(past_time, past_angle, label='pendulum angle (deg)', color='red')
    plt.plot(past_time, target_line, label='Baseline', color='blue')

    plt.title('Furata Pendulum')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.legend()
    plt.grid(True)
    plt.show()



        