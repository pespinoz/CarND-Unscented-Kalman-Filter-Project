import pandas as pd
import numpy as np
import itertools
import matplotlib.pyplot as plt
plt.interactive(True)


l_sensor_type, l_px_meas, l_py_meas, l_time_meas, l_px_gt, l_py_gt, l_vx_gt, l_vy_gt, l_yaw_gt, l_yaw_dot_gt = [], [], [], [], [], [], [], [], [], []
with open('data/laser_radar_data.txt') as f:
    for laser_line in itertools.islice(f, 0, None, 2):
        aux = laser_line.split(',')
        l_sensor_type.append(aux[0])
        l_px_meas.append(float(aux[1]))
        l_py_meas.append(float(aux[2]))
        l_time_meas.append(float(aux[3]))
        l_px_gt.append(float(aux[4]))
        l_py_gt.append(float(aux[5]))
        l_vx_gt.append(float(aux[6]))
        l_vy_gt.append(float(aux[7]))
        l_yaw_gt.append(float(aux[8]))
        l_yaw_dot_gt.append(float(aux[9]))

r_sensor_type, r_rho_meas, r_theta_meas, r_rho_dot_meas, r_time_meas, r_px_gt, r_py_gt, r_vx_gt, r_vy_gt, r_yaw_gt, r_yaw_dot_gt = [], [], [], [], [], [], [], [], [], [], []
with open('data/laser_radar_data.txt') as g:
    for radar_line in itertools.islice(g, 1, None, 2):
        aux = radar_line.split(',')
        r_sensor_type.append(aux[0])
        r_rho_meas.append(float(aux[1]))
        r_theta_meas.append(float(aux[2]))
        r_rho_dot_meas.append(float(aux[3]))
        r_time_meas.append(float(aux[4]))
        r_px_gt.append(float(aux[5]))
        r_py_gt.append(float(aux[6]))
        r_vx_gt.append(float(aux[7]))
        r_vy_gt.append(float(aux[8]))
        r_yaw_gt.append(float(aux[9]))
        r_yaw_dot_gt.append(float(aux[10]))

state = pd.read_csv('state-vec_and_stats.txt', sep=',', header=None, names=['px', 'py', 'v', 'yaw', 'yaw_dot', 'nis',
                                                                            'rmse_x', 'rmse_y', 'rmse_vx', 'rmse_vy'])

################################################################

# sort the times measured
time_meas = np.array(l_time_meas + r_time_meas)
sort_index = np.argsort(time_meas)

# merge the (lidar and radar) measurements and ground truths
sensor = np.array(l_sensor_type + r_sensor_type)
px_meas = np.concatenate((np.array(l_px_meas), r_rho_meas*np.cos(r_theta_meas)))
py_meas = np.concatenate((np.array(l_py_meas), r_rho_meas*np.sin(r_theta_meas)))
px_gt = np.array(l_px_gt + r_px_gt)
py_gt = np.array(l_py_gt + r_py_gt)
vx_gt = np.array(l_vx_gt + r_vx_gt)
vy_gt = np.array(l_vy_gt + r_vy_gt)
yaw_gt = np.array(l_yaw_gt + r_yaw_gt)
yaw_dot_gt = np.array(l_yaw_dot_gt + r_yaw_dot_gt)

# with the computed index I sort the rest of the variables
sensor = sensor[sort_index]
time_meas = time_meas[sort_index]
px_meas = px_meas[sort_index]
py_meas = py_meas[sort_index]
px_gt = px_gt[sort_index]
py_gt = py_gt[sort_index]
vx_gt = vx_gt[sort_index]
vy_gt = vy_gt[sort_index]
yaw_gt = yaw_gt[sort_index]
yaw_dot_gt = yaw_dot_gt[sort_index]

################################################################

#  px vs py
fig = plt.figure(figsize=(14, 5))
plt.plot(state.px, state.py, color='r')
plt.plot(px_gt, py_gt, color='b')
plt.plot(px_meas, py_meas, color='g')
plt.xlabel('px in m', fontsize=15)
plt.ylabel('py in m', fontsize=15)
plt.legend(['UKF Position Estimation', 'Ground Truth Position', 'Position Measurements'], loc='lower right', fontsize=11)
plt.xlim([-27.5, -3])
plt.ylim([-10.5, 2.5])
plt.show()
fig.savefig('figures/py_vs_px.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)

# t vs v
fig = plt.figure(figsize=(14, 5))
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.v, color='r')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), np.sqrt(np.square(vx_gt) + np.square(vy_gt)), color='b')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('v in m/s', fontsize=15)
plt.legend(['UKF Velocity Estimation', 'Ground Truth Velocity'], loc='lower right', fontsize=11)
plt.xlim([-0.2, 25.1])
plt.show()
fig.savefig('figures/v_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)

# t vs yaw
fig = plt.figure(figsize=(14, 5))
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.yaw, color='r')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), yaw_gt, color='b')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('yaw in rad', fontsize=15)
plt.legend(['UKF Yaw Estimation', 'Ground Truth Yaw'], loc='upper right', fontsize=11)
plt.xlim([-0.2, 25.1])
plt.show()
fig.savefig('figures/yaw_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)

# t vs yaw rate
fig = plt.figure(figsize=(14, 5))
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.yaw_dot, color='r')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), yaw_dot_gt, color='b')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('yaw in rad/s', fontsize=15)
plt.legend(['UKF Yaw Rate Estimation', 'Ground Truth Yaw Rate'], loc='upper right', fontsize=11)
plt.xlim([-0.2, 25.1])
plt.show()
fig.savefig('figures/yawrate_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)


nis = np.array(state.nis)
index_r = np.array(np.where(sensor == 'R'))
index_l = np.array(np.where(sensor == 'L'))

xx = (time_meas[index_r]/1000000)-(time_meas[0]/1000000)
yy = nis[index_r]
cte = 7.815  #(chi square 95% with three degrees of freedom)
# t vs nis radar
fig = plt.figure(figsize=(14, 5))
plt.plot(xx[0], yy[0], color='r')
plt.plot((0, 26), (cte, cte), 'k-')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('nis radar', fontsize=15)
plt.show()
fig.savefig('figures/nisradar_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)

xx = (time_meas[index_l]/1000000)-(time_meas[0]/1000000)
yy = nis[index_l]
cte = 5.991  # (chi square 95% with two degrees of freedom)
# t vs nis laser
fig = plt.figure(figsize=(14, 5))
plt.plot(xx[0], yy[0], color='r')
plt.plot((0, 26), (cte, cte), 'k-')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('nis laser', fontsize=15)
plt.xlim([-0.2, 25.1])
plt.show()
fig.savefig('figures/nislaser_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)

# t vs rmse
fig = plt.figure(figsize=(14, 5))
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.rmse_x, color='r', linestyle='-')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.rmse_y, color='r', linestyle='--')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.rmse_vx, color='g', linestyle='-')
plt.plot((time_meas/1000000)-(time_meas[0]/1000000), state.rmse_vy, color='g', linestyle='--')
plt.xlabel('t in s', fontsize=15)
plt.ylabel('RMSE', fontsize=15)
plt.legend(['rmse px', 'rmse py', 'rmse vx', 'rmse vy'], loc='upper right', fontsize=11)
plt.xlim([-0.2, 25.1])
plt.ylim([0, 3.2])
plt.show()
fig.savefig('figures/rmse_vs_t.jpg', dpi=None, facecolor='w', edgecolor='w',
            orientation='portrait', papertype=None, format=None, transparent=False, bbox_inches='tight',
            pad_inches=0.1, frameon=None)
