#woohoo another soham project
#python file to mujoco sim - sample pid testing
# this simulation took like - 20ish hours or more to get working :|

import mujoco 
import mujoco.viewer
import matplotlib.pyplot as plt
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("furatamjcf.xml")
data = mujoco.MjData(model)

bias = 0

#creating a controller class for easiness
class PID:
    def __init__ (self, kp, ki, kd): #initializing these parameters (pid values) since this will apply to both control loops
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.last_time = 0

    def calculation(self, target, current_deg, current_time, modulus): # main calculation class, basically just uses init values and produces an
        if self.last_time == 0:
            self.last_time = current_time
        

        deltatime = current_time - self.last_time
        if modulus == True:
            error = (target - current_deg + 180) % 360 - 180
        else:
            error = (target - current_deg)

        if deltatime > 0:
           derivative = (error - self.last_error)/deltatime
        else:
            derivative = 0

        
        self.integral += error*deltatime
       

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd*derivative) + bias

        self.last_error = error
        self.last_time = current_time

        return output

data.joint("pend").qpos[0] = np.radians(165)

arm_pid = PID(kp = 1, ki = 0.025, kd = 0.15) #working pid values
pend_pid = PID(kp = 50, ki = 0.40, kd = 0.35)

# an oscillatory behaviour will remain here, since this system is basically oscillating between 2 events out of phase

past_time, past_pend, past_arm = [], [], []

steps = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():


        current_time = data.time

        current_pend_angle = np.degrees(data.joint("pend").qpos[0] % 360)
        current_arm_angle = np.degrees(data.joint("arm").qpos[0])

        
        
        pend_output = pend_pid.calculation(180, current_pend_angle, current_time, True)
        arm_output = arm_pid.calculation(0, current_arm_angle, current_time, False)
        data.actuator("n20").ctrl[0] = np.clip(-(arm_output + pend_output), -1, 1)

        mujoco.mj_step(model, data)
        viewer.sync()

        steps += 1

        if steps % 10 == 0:
            steps = 0
            past_time.append(current_time)
            past_pend.append(current_pend_angle)
            past_arm.append(current_arm_angle)



fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax1.plot(past_time, past_pend, color="red", label="Pendulum Angle")
ax1.axhline(y=180, color='black', linestyle='--', label="Target")
ax1.set_ylabel("Degrees")
ax1.legend()
ax1.grid(True)

ax2.plot(past_time, past_arm, color="blue", label="Arm Position")
ax2.axhline(y=0, color='black', linestyle='--', label="Target")
ax2.set_ylabel("Degrees")
ax2.set_xlabel("Time")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()
