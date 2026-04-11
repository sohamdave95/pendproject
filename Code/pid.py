import mujoco 
import mujoco.viewer
import matplotlib.pyplot as plt
import numpy as np
import os
import time

# --- PATH CONFIG ---
script_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(script_dir, "furatamjcf.xml")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.last_error = 0

    def compute(self, current, target, dt):
        # Normalize error to [-180, 180]
        error = (current - target + 180) % 360 - 180
        
        # Integral with anti-windup
        self.integral = np.clip(self.integral + (error * dt), -1, 1)
        
        # Derivative (Damping)
        derivative = (error - self.last_error) / dt
        self.last_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return np.clip(output, -1, 1)

# --- IRL TUNING FOR 3.5g MASS ---
# Low mass requires very low P to avoid high-frequency jitter.
# High D helps stabilize the feather-weight pendulum.
pid = PIDController(kp=40.0, ki=0.0, kd=12.0)

# Init Pendulum near upright (180 deg)
data.joint("pend").qpos[0] = np.radians(185) 
target_angle = 180 

history_time, history_pend, history_arm = [], [], []

print("Launching Simulation...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running() and data.time < 10.0:
        step_start = time.time()

        # 1. State Update
        angle_pend = np.degrees(data.joint("pend").qpos[0])
        angle_arm = np.degrees(data.joint("arm").qpos[0])

        # 2. Control (Sign Check: - signal usually corrects falling toward + angle)
        # Change '-' to '+' if arm moves the wrong way in sim
        control_signal = pid.compute(angle_pend, target_angle, model.opt.timestep)
        data.actuator("n20").ctrl[0] = control_signal

        # 3. Physics Step
        mujoco.mj_step(model, data)
        
        # 4. Logging & Visuals
        history_time.append(data.time)
        history_pend.append(angle_pend)
        history_arm.append(angle_arm)
        
        viewer.sync()
        
        # Sync to real-world time
        time_diff = model.opt.timestep - (time.time() - step_start)
        if time_diff > 0:
            time.sleep(time_diff)

# --- MATPLOTLIB ---
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(history_time, history_pend, color='red', label='Pendulum Angle')
plt.axhline(y=180, color='blue', linestyle='--')
plt.ylabel('Degrees'); plt.legend(); plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(history_time, history_arm, color='green', label='Arm Angle (Gear 30)')
plt.xlabel('Time (s)'); plt.ylabel('Degrees'); plt.legend(); plt.grid(True)

plt.tight_layout()
plt.show()