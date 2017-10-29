This directory is not included in the original repository.
The data is read through the simulator (but it's the same data)

obj_pose-laser-radar-synthetic-input.txt comes from EKF project
laser_radar_data.txt comes from executing ./UKF	

-----------------------------------------------------------------------------
From Project UKF, slide 2) File Structure

The data file information is provided by the simulator and is the same data files from EKF. Again each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.

Although the data set contains values for yaw and yawrate ground truth, there is no need to use these values. main.cpp does not use these values, and you are only expected to calculate RMSE for x, y vx and vy. You can compare your vx and vy RMSE values from the UKF project and the EKF project. For UKF, vx and vy RMSE should be lower than for EKF; this is because we are using a more detailed motion model and UKF is also known for handling non-linear equations better than EKF.
