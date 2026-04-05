clear; clc; close all;

% Simulation parameters
dt = 0.01;           % Time step [s]
T = 60;              % Total time [s]
N = round(T/dt);     % Number of steps

% Robot initial state
x = 0; y = 0; theta = 0;

% Store trajectory
X = zeros(N,1); Y = zeros(N,1); THETA = zeros(N,1);
X_d = zeros(N,1); Y_d = zeros(N,1);

% Controller gains
k1 = 1.0; k2 = 5.0; k3 = 2.0;

% Desired trajectory (circular)
radius = 5;
omega_d = 0.2;
v_d = radius * omega_d;

% Tractor vehicle dimensions
l = 2.4;       % length of tractor base
w = 1.2;       % width of tractor base

% Cabin dimensions
cab_l = 0.8;   
cab_w = 0.9;
cab_x_offset = -0.3; % relative to body center

% Hood
hood_l = 0.5;
hood_w = 0.6;
hood_x_offset = l/2 - hood_l/2;

% Rear Wheels
rear_wheel_r = 0.5; % radius
rear_wheel_w = 0.3; % width

% Front Wheels
front_wheel_r = 0.25;
front_wheel_w = 0.2;

% Rear Wheel Positions
rear_wheel_pos = [-l/2 + 0.2,  w/2 + rear_wheel_w/2;
                  -l/2 + 0.2, -w/2 - rear_wheel_w/2];

% Front Wheel Positions (projecting slightly outside)
front_wheel_pos = [l/2 - 0.3,  l/2 - 0.3;
                   w/2 + 0.1, -w/2 - 0.1];

% Create figure for animation
figure(1);
tic;
for i = 1:N
    t = (i-1)*dt;
    
    % Desired trajectory
    x_d = radius * cos(omega_d * t);
    y_d = radius * sin(omega_d * t);
    theta_d = atan2(y_d - y, x_d - x);  % Optional for reference
    dx_d = -radius * omega_d * sin(omega_d * t);
    dy_d =  radius * omega_d * cos(omega_d * t);
    ddx_d=-radius * omega_d * omega_d *cos(omega_d * t);
    ddy_d=-radius * omega_d * omega_d *sin(omega_d * t);
    
    v_d_current = sqrt(dx_d^2 + dy_d^2);
    omega_d_current = (dx_d * ddy_d - dy_d * ddx_d) / (dx_d^2 + dy_d^2 ); % approximate

    % Compute control inputs
    [v, omega] = unicycle_tracking_control(x, y, theta, ...
        x_d, y_d, theta_d, v_d_current, omega_d_current, k1, k2, k3);
    
    % Robot kinematics update
    x = x + dt * v * cos(theta);
    y = y + dt * v * sin(theta);
    theta = theta + dt * omega;
    theta = wrapToPi(theta);

    % Log data
    X(i) = x; Y(i) = y; THETA(i) = theta;
    X_d(i) = x_d; Y_d(i) = y_d;

    %% Tractor Animation

    R_psi = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Body
    body_shape = [-l/2, l/2, l/2, -l/2, -l/2; -w/2, -w/2, w/2, w/2, -w/2];
    body_pos = R_psi * body_shape;
    fill(body_pos(1,:) + x, body_pos(2,:) + y, [0.3 0.6 0.9]); hold on; % blue base

    % Cabin
    cabin_shape = [-cab_l/2, cab_l/2, cab_l/2, -cab_l/2, -cab_l/2;
                   -cab_w/2, -cab_w/2, cab_w/2, cab_w/2, -cab_w/2];
    cabin_center = R_psi * [cab_x_offset; 0];
    cabin_pos = R_psi * cabin_shape;
    fill(cabin_pos(1,:) + x + cabin_center(1), cabin_pos(2,:) + y + cabin_center(2), [0.8 0.8 0.2]); % yellow cabin

    % Hood
    hood_shape = [-hood_l/2, hood_l/2, hood_l/2, -hood_l/2, -hood_l/2;
                  -hood_w/2, -hood_w/2, hood_w/2, hood_w/2, -hood_w/2];
    hood_center = R_psi * [hood_x_offset; 0];
    hood_pos = R_psi * hood_shape;
    fill(hood_pos(1,:) + x + hood_center(1), hood_pos(2,:) + y + hood_center(2), [0.5 0.5 0.5]); % gray hood

    % Rear Wheels
    for k = 1:2
        pos = rear_wheel_pos(k,:)';
        wheel_center = R_psi * pos;
        theta_wheel = linspace(0, 2*pi, 30);
        fill(rear_wheel_r * cos(theta_wheel) + x + wheel_center(1), ...
             rear_wheel_r * sin(theta_wheel) + y + wheel_center(2), 'k');
    end

    % Front Wheels
    for k = 1:2
        pos = front_wheel_pos(:,k);
        wheel_center = R_psi * pos;
        theta_wheel = linspace(0, 2*pi, 30);
        fill(front_wheel_r * cos(theta_wheel) + x + wheel_center(1), ...
             front_wheel_r * sin(theta_wheel) + y + wheel_center(2), [0.2 0.2 0.2]);
    end

    % Heading arrow
    heading_length = 2.8;
    quiver(x, y, heading_length * cos(theta), heading_length * sin(theta), 'm', 'LineWidth', 3, 'MaxHeadSize', 2);

    % Path and reference
    plot(X(1:i), Y(1:i), 'b-', 'LineWidth', 2);
    plot(X_d, Y_d, 'r--', 'LineWidth', 1.5); hold on;

    % Title and axis labels
    elapsed_time = toc;  % Get elapsed time in seconds
    title(sprintf('Time elapsed: %.2f seconds', elapsed_time), 'Interpreter', 'latex', 'FontSize', 12);
    xlabel('$x~[m]$', 'Interpreter', 'latex', 'FontSize', 16);
    ylabel('$y~[m]$', 'Interpreter', 'latex', 'FontSize', 16);
    axis([-8 8 -8 8]); axis square
    grid on; box on;
    set(gcf,'color','w'); set(gca,'FontSize',16); set(gca,'TickLabelInterpreter','latex');
    pause(0.01); hold off;
end

% Unicycle tracking control function
function [v, omega] = unicycle_tracking_control(x, y, theta, x_d, y_d, theta_d, v_d, omega_d, k1, k2, k3)
    % Compute position errors
    dx = x_d - x;
    dy = y_d - y;

    % Transform errors to body frame
    e_x = cos(theta) * dx + sin(theta) * dy;
    e_y = -sin(theta) * dx + cos(theta) * dy;
    e_theta = wrapToPi(theta_d - theta);

    % Control laws
    v = v_d * cos(e_theta) + k1 * e_x;
    omega = omega_d + k2 * e_y + k3 * sin(e_theta);
end
