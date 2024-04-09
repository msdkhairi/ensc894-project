clear all; close all; clc;

way_points = [59.99 -38.88 -190 11.11;
              57.8411 -85.1201 -170 -37.2791;
              37.9206 -95.4803 -150 -67.5597;
              10.8439 -85.1201 -130 -84.2762;
              -27.4639 -38.8804 -100 -76.3443];

time_points = [9.504 14.504 19.504 24.504 29.504];
time_interval = [9.504 29.504];

t_samples = linspace(9.5, 29.5, 1000);


%[q, qd, qdd, pp] = cubicpolytraj(way_points.', time_points, t_samples);
[q, qd, qdd, pp] = bsplinepolytraj(way_points.', time_interval, t_samples);


% Plot the trajectory, velocity, and acceleration profiles
figure;

subplot(3,1,1);
plot(t_samples, q);
xlabel('Time (s)');
ylabel('Joint Position (deg)');
title('Single Polynomial Trajectory');

subplot(3,1,2);
plot(t_samples, qd);
xlabel('Time (s)');
ylabel('Joint Velocity (deg/s)');
title('Single Polynomial Velocity Profile');

subplot(3,1,3);
plot(t_samples, qdd);
xlabel('Time (s)');
ylabel('Joint Acceleration (deg/s^2)');
title('Single Polynomial Acceleration Profile');