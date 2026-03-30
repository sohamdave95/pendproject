#woohoo another soham project
#python file to mujoco sim - sample pid testing
#uses class to simplify down the math - and to save in the memory which is needed since we use prior error value in memory each time
#this took me 14 hours to get to work, im going insane.

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
past_base = []


# PID values
kp = 175 #150 or 175
ki = 0 #0 or 0.5
kd = 25 #25 or 30



class pid_loop:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0


    def pidcalcs(self, currentangle, dt):
        #first thing we do which is read the angle
        error = currentangle - targetangle
        
        error = (error + 180) % 360 - 180 # Forces error between -180 and 180

        #tracking derivative and integral terms over time
        self.integral += error*dt
        deriv = (error - self.last_error)/dt

        output = (self.kp*error) + (self.ki*self.integral) + (self.kd*deriv)
        self.last_error = error

        return numpy.clip(output, -1, 1)


pid = pid_loop(kp,ki,kd)

data.joint("pend").qpos[0] = numpy.radians(180) #originally setting this so i can test the sim at an original angle
mujoco.mj_forward(model, data)


with mujoco.viewer.launch_passive(model, data) as viewer:




    while viewer.is_running():


        step_start = time.time()


        base_deg = numpy.degrees(data.joint("arm").qpos[0])
        currentangle = numpy.degrees(data.joint("pend").qpos[0] % 360)



        elapsed = time.time() - step_start
        if elapsed < timestep:
            time.sleep(timestep - elapsed)

        if base_deg > 350:
            targetangle = 165
        elif base_deg < -350:
            targetangle = 195
        else:
            targetangle = 180

        signal = pid.pidcalcs(currentangle, model.opt.timestep)
        data.actuator("n20").ctrl[0] = signal


        mujoco.mj_step(model, data)
        viewer.sync()

        #for matplotlib visualizing - append prior data to the error for graph
        current_time = data.time
        past_time.append(current_time)
        past_angle.append(currentangle) 
        past_base.append(base_deg)

    plt.figure(figsize=(10, 5))
    plt.plot(past_time, past_angle, label='pendulum angle (deg)', color='red')
    #plt.plot(past_time, past_base, label="base angle (deg)", color="blue")
    plt.axhline(y=180, label='Baseline', color='blue', linestyle='--')

    plt.title('Furata Pendulum')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.legend()
    plt.grid(True)
    plt.show()