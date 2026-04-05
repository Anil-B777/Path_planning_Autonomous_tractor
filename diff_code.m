clc; clear; close all;

% Parameters
dt = 0.01;
T = 50;
steps = T / dt;

% Control gains
k1 = 20.0;
k2 =6.0;

% Initialize state
x = 0; y = 0; theta = 0;

% Log
x_traj = zeros(1, steps);
y_traj = zeros(1, steps);
x_ref = zeros(1, steps);
y_ref = zeros(1, steps);

for i = 1:steps
    t = (i-1)*dt;
    
    % Desired trajectory: Circle
    R = 2; w = 0.2;               % radius and angular speed
    x_d = R * cos(w * t);
    y_d = R * sin(w * t);
    theta_d = atan2(y_d - R * sin(w*t - dt), x_d - R * cos(w*t - dt));  % numerical derivative approx
    v_d = R * w;                  % constant tangential speed
    omega_d = w;                  % constant angular velocity

    % Errors in body frame
    dx = x_d - x;
    dy = y_d - y;
    e_x = cos(theta)*dx + sin(theta)*dy;
    e_y = -sin(theta)*dx + cos(theta)*dy;
    e_theta = wrapToPi(theta_d - theta);

    % Control law
    v = v_d;  % match desired speed exactly
    omega = omega_d + k1 * e_y + k2 * sin(e_theta);

    % Update robot state
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    % Log
    x_traj(i) = x;
    y_traj(i) = y;
    x_ref(i) = x_d;
    y_ref(i) = y_d;
end

% Plot
figure;
plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(x_traj, y_traj, 'b-', 'LineWidth', 2);
legend('Desired Path', 'Tracked Path');
xlabel('x'); ylabel('y'); axis equal; grid on;
title('Improved Unicycle Trajectory Tracking');
