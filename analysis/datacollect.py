import matplotlib.pyplot as plt 
import numpy as np
from statistics import mean 

import serial
ser = serial.Serial('COM7')
print('Opening port: ')
print(ser.name)

# read the user's choice
selection = input('\nInput s to Start: ')
selection_endline = selection+'\n'
     
# send the command to the PIC32
ser.write(selection_endline.encode()); # .encode() turns the string into a char array



n_int = 3000 # turn it into an int
print('Data length = ' + str(n_int))
theta = []
theta_dot = []
ro = []
i = []
d = []
xhat = np.empty((4, 0))
data_received = 0
while data_received < n_int:
    dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
    dat_f = list(map(float,dat_str.split())) # now the data is a list
    if len(dat_f) != 9:
        print("Unexpected line:", dat_str)
        continue
    theta.append(dat_f[0])
    theta_dot.append(dat_f[1])
    ro.append(dat_f[2])
    i.append(dat_f[3])
    d.append(dat_f[4])
    xdat = np.array(dat_f[5:9]).reshape(4, 1)
    xhat = np.hstack((xhat, xdat))
    data_received = data_received + 1
    
t = np.arange(0,n_int*0.002, 0.002) # index array

fig, ax = plt.subplots(4, 1, sharex=True)
ax[0].plot(t, theta, label='theta')
ax[0].plot(t, xhat[0, :], label='theta hat')
ax[0].legend()
ax[0].set_ylabel('rad')

ax[1].plot(t, theta_dot, label='omega')
ax[1].plot(t, xhat[1, :], label='omega hat')
ax[1].legend()
ax[1].set_ylabel('rad/s')

ax[2].plot(t, ro, label='ro')
ax[2].plot(t, xhat[2, :], label='ro hat')
ax[2].legend()
ax[2].set_ylabel('rad/s')

ax[3].plot(t, i, label='current')
ax[3].plot(t, xhat[3, :], label='current hat')
ax[3].legend()
ax[3].set_ylabel('A')
ax[3].set_xlabel('Time (s)')


idx = t > 3.0
theta = np.array(theta)
theta_dot = np.array(theta_dot)
ro = np.array(ro)
i = np.array(i)
duty = np.array(d)
print("mean theta:", np.mean(theta[idx]))
steady_mean = np.mean(theta[idx])
steady_rmse = np.sqrt(np.mean(theta[idx]**2))
steady_max = np.max(np.abs(theta[idx]))
print("mean rho:", np.mean(ro[idx]))
print("mean current:", np.mean(i[idx]))
print("mean duty:", np.mean(duty[idx]))

# full interval
theta_err = theta - xhat[0, :]
rho_err = ro - xhat[2, :]

# settled observer interval, for example after 2 seconds
idx = t > 3.0

print("FULL theta RMSE:", np.sqrt(np.mean(theta_err**2)))
print("FULL theta max:", np.max(np.abs(theta_err)))

print("STEADY theta RMSE:", np.sqrt(np.mean(theta_err[idx]**2)))
steady_theta_rmse = np.sqrt(np.mean(theta_err[idx]**2))
print("STEADY theta bias:", np.mean(theta_err[idx]))
print("STEADY theta max:", np.max(np.abs(theta_err[idx])))
steady_theta_max =  np.max(np.abs(theta_err[idx]))

print("STEADY rho RMSE:", np.sqrt(np.mean(rho_err[idx]**2)))
print("STEADY rho bias:", np.mean(rho_err[idx]))

print("rho std:", np.std(ro[idx]))
print("rho_hat std:", np.std(xhat[2, idx]))

ro_std = np.std(ro[idx])
roh_std = np.std(xhat[2,idx])
var_redux = ((ro_std - roh_std)/ro_std)*100.0


annotation = (
    f"Steady-state RMSE: {np.degrees(steady_theta_rmse):.3f}°\n"
    f"Max error: {np.degrees(steady_theta_max):.3f}°"
)

annotation2 = (
    f"Steady-state Mean Error: {np.degrees(steady_mean):.3f}°\n"
    f"Steady-state RMSE: {np.degrees(steady_rmse):.3f}°\n"
    f"Steady-state Max Error: {np.degrees(steady_max):.3f}°"
)

annotation3 = (
    rf'$\rho$ std: {ro_std:.3f} rad/s' '\n'
    rf'$\hat{{\rho}}$ std: {roh_std:.3f} rad/s' '\n'
    f'Variation reduction: {var_redux:.1f}%'
)

plt.figure()
plt.plot(t, theta, label= r'$\theta$')
plt.plot(t, xhat[0, :], label=r'$\hat{\theta}$')
plt.axhline(0, linestyle='--', linewidth=1)
plt.text(
    0.98, 0.08,
    annotation,
    transform=plt.gca().transAxes,
    ha='right',
    va='bottom'
)
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angle (Radians)')
plt.title(r'Measured $\theta$ vs. Estimated $\hat{\theta}$')
plt.grid()

plt.figure()
plt.plot(t, theta, label= r'$\theta$')
plt.axhline(0, linestyle='--', linewidth=1)
plt.text(
    0.98, 0.08,
    annotation2,
    transform=plt.gca().transAxes,
    ha='right',
    va='bottom'
)
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angle (Radians)')
plt.title(r'Measured $\theta$ During Catch and Steady State')
plt.grid()

plt.figure()
plt.plot(t, ro, label= r'$\rho$')
plt.plot(t, xhat[2, :], label=r'$\hat{\rho}$')
plt.axhline(0, linestyle='--', linewidth=1)
plt.text(
    0.98, 0.08,
    annotation3,
    transform=plt.gca().transAxes,
    ha='right',
    va='bottom'
)
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angle (Radians)')
plt.title(r'Flywheel Speed Estimation: $\rho$ vs. $\hat{\rho}$')
plt.grid()

plt.show()

