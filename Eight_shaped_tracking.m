clear; clc; close all;

% Tractor details

 a1 = 0.15; a2 = 0.08;
 t1 = 0.06; t2 = 0.04;
 D = 0.2; d = 0.1; l = 0.5; b = 0.10;
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
% k1 = 1.0; k2 = 5.0; k3 = 2.0;
 k1 = 1.0; k2 = 2; k3 = 2;
% Desired trajectory (circular)
radius = 2;
omega_d = 0.2;
% v_d = radius * omega_d;
r1=5;r2=3;
for i = 1:N
    t = (i-1)*dt;
    
    % Desired trajectory
    x_d = r1* sin(omega_d * t);
    y_d = (r2/2) * sin(2*omega_d * t);
%     theta_d = atan2(y_d - y, x_d - x);  
    dx_d = r1*omega_d *cos(omega_d * t);
    dy_d =  r2 * omega_d * cos(2*omega_d * t);
    ddx_d = -r1 * omega_d ^2* sin(omega_d * t);
    ddy_d =  -2*r2* omega_d ^2* sin(2*omega_d * t);
    theta_d = atan2(dy_d, dx_d);
    v_d_current = sqrt(dx_d^2 + dy_d^2);
    omega_d_current = (ddy_d * dx_d - ddx_d * dy_d) / (dx_d^2 + dy_d^2 + 1e-6); % approximate

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
    phi(i) = atan(l*omega/v);
end

% Plotting
figure;
tic
plot(X, Y, 'b-', 'LineWidth', 3);hold on;
plot(X_d, Y_d, 'r--', 'LineWidth', 2); 

legend('Desired', 'Actual');
xlabel('x'); ylabel('y');
title('Unicycle Trajectory Tracking');
axis equal; grid on;

veh_box = [-b,b,b,l,l,b,b,-b,-b;-D,-D,-d,-d,d,d,D,D,-D];
wheel_s = [-a2,a2,a2,-a2,-a2;-t2/2,-t2/2,t2/2,t2/2,-t2/2];
wheel_b = [-a1,a1,a1,-a1,-a1;-t1/2,-t1/2,t1/2,t1/2,-t1/2];

for i = 1:5:length(X)
    psi = THETA(i); 
    R = [cos(psi),-sin(psi);sin(psi),cos(psi)];
    v_m = R*veh_box;
    w_m3 = R*(wheel_b+[0;D+t1/2]);
    w_m4 = R*(wheel_b+[0;-D-t1/2]);
    R1 = [cos(phi(i)),-sin(phi(i));sin(phi(i)),cos(phi(i))];

    w_m1 = R*(R1*wheel_s+[l;d+t2/2]);
    w_m2 = R*(R1*wheel_s+[l;-d-t2/2]);
    fill(v_m(1,:)+X(i),v_m(2,:)+Y(i),'y');
    hold on
    fill(w_m1(1,:)+X(i),w_m1(2,:)+Y(i),'r');
    fill(w_m2(1,:)+X(i),w_m2(2,:)+Y(i),'r');
    fill(w_m3(1,:)+X(i),w_m3(2,:)+Y(i),'g');
    fill(w_m4(1,:)+X(i),w_m4(2,:)+Y(i),'g');
    plot(X_d,Y_d,'m--','LineWidth',2)
    plot(X(1:i),Y(1:i),'b--','LineWidth',2)
    elapsed_time = toc;  % Get elapsed time in seconds
    title(sprintf('Time elapsed: %.2f seconds', elapsed_time),'Interpreter','latex','FontSize',12);
    set(gca, 'fontsize', 14)
    set(gca, 'TickLabelInterpreter','latex');
    set(gca, 'FontSize', 16); set(gcf,'color','w'); grid on; box on;
    
    xmin = min(X) - 0.5;
    xmax = max(X) + 0.5;
    ymin = min(Y) - 0.5;
    ymax = max(Y) + 0.5;
    axis([xmin xmax ymin ymax])
    axis equal
    grid on
    xlabel('x,[m]')
    ylabel('y,[m]');
    pause(0.01)
    hold off
end



function [v, omega] = unicycle_tracking_control(x, y, theta, x_d, y_d, theta_d, v_d, omega_d, k1, k2, k3)
    % Compute position errors
    dx = x_d - x;
    dy = y_d - y;

    % Transform errors to body frame
    e_x = cos(theta) * dx + sin(theta) * dy;
    e_y = -sin(theta) * dx + cos(theta) * dy;
    e_theta = wrapToPi(theta_d - theta);

    % Control laws
%       v = v_d * cos(e_theta) + k1 * e_x;
  v=v_d;
    omega = omega_d + k2 * e_y + k3 * sin(e_theta);
end




