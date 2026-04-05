clear; clc; close all;

% Tractor details

 a1 = 0.15; a2 = 0.08;
 t1 = 0.06; t2 = 0.04;
 D = 0.2; d = 0.1; l = 0.5; b = 0.10;
% Simulation parameters
dt = 0.01;           % Time step [s]
T = 200;              % Total time [s]
N = round(T/dt);     % Number of steps

% Robot initial state
x = 0; y = 1; theta = 0;

% Store trajectory
X = zeros(N,1); Y = zeros(N,1); THETA = zeros(N,1);
X_d = zeros(N,1); Y_d = zeros(N,1);
X_f = zeros(N,1); Y_f = zeros(N,1);

% Controller gains
% k1 = 1.0; k2 = 5.0; k3 = 2.0;
 k1 = 1.0; k2 = 2; k3 = 5;
tic
for i = 1:N
    t = (i-1)*dt;
    time(i)=t;

if t < 20
    eta_f = [0; 1; 0];
    eta_f_dot = [0; 0; 0];
    eta_f_ddot = [0; 0; 0];

    eta_d = [0; 1; 0];
    eta_d_dot = [0; 0; 0];
    eta_d_ddot = [0; 0; 0];

elseif t < 40
    eta_f = [0.2*(t - 20); 1; 0];
    eta_f_dot = [0.2; 0; 0];
    eta_f_ddot = [0; 0; 0];

   eta_d = [0.2*(t - 20); 1; 0];
    eta_d_dot = [0.2; 0; 0];
    eta_d_ddot = [0; 0; 0];


elseif t < 60
    eta_f = [sin(0.05*pi*(t - 40)) + 4; -cos(0.05*pi*(t - 40)) + 2; 0];
    eta_f_dot = [0.05*pi*cos(0.05*pi*(t - 40)); 0.05*pi*sin(0.05*pi*(t - 40)); 0];
    eta_f_ddot = [-(0.05*pi)^2*sin(0.05*pi*(t - 40)); (0.05*pi)^2*cos(0.05*pi*(t - 40)); 0];
     
   thet = -pi/2 + (pi/20)*(t - 40);  
    eta_d = [4 + 2*cos(thet); 3 + 2*sin(thet); 0];
    eta_d_dot = [-2*(pi/20)*sin(thet); 2*(pi/20)*cos(thet); 0];
    eta_d_ddot = [-2*(pi/20)^2*cos(thet); -2*(pi/20)^2*sin(thet); 0];
    

elseif t < 80
    eta_f = [-0.2*(t - 60) + 4; 3; 0];
    eta_f_dot = [-0.2; 0; 0];
    eta_f_ddot = [0; 0; 0];

     eta_d = [-0.2*(t - 60) + 4; 5; 0];
    eta_d_dot = [-0.2; 0; 0];
    eta_d_ddot = [0; 0; 0];

elseif t < 100
    eta_f = [-sin(0.05*pi*(t - 80)); -cos(0.05*pi*(t - 80)) + 4; 0];
    eta_f_dot = [-0.05*pi*cos(0.05*pi*(t - 80)); 0.05*pi*sin(0.05*pi*(t - 80)); 0];
    eta_f_ddot = [(0.05*pi)^2*sin(0.05*pi*(t - 80)); (0.05*pi)^2*cos(0.05*pi*(t - 80)); 0];

    thet = -pi/2 + (pi/20)*(t - 80);  % angle from -π/2 to π/2
    eta_d = [0 - 1*cos(thet);7+2*sin(thet); 0];  % center at (0,7)
    eta_d_dot = [1*(pi/20)*sin(thet); 2*(pi/20)*cos(thet); 0];
    eta_d_ddot = [1*(pi/20)^2*cos(thet); -2*(pi/20)^2*sin(thet); 0];




elseif t < 120
    eta_f = [0.2*(t - 100); 5; 0];
    eta_f_dot = [0.2; 0; 0];
    eta_f_ddot = [0; 0; 0];

     eta_d = [0.2*(t - 100); 9; 0];
    eta_d_dot = [0.2; 0; 0];
    eta_d_ddot = [0; 0; 0];

%% --- SECOND CYCLE STARTS ---

elseif t < 140
    % Circular arc from [4,5] to [4,7]
    eta_f = [sin(0.05*pi*(t - 120)) + 4; -cos(0.05*pi*(t - 120)) + 6; 0];
    eta_f_dot = [0.05*pi*cos(0.05*pi*(t - 120)); 0.05*pi*sin(0.05*pi*(t - 120)); 0];
    eta_f_ddot = [-(0.05*pi)^2*sin(0.05*pi*(t - 120)); (0.05*pi)^2*cos(0.05*pi*(t - 120)); 0];

      eta_d = [4; 9; 0];
        eta_d_dot = [0; 0; 0];
        eta_d_ddot = [0; 0; 0];


elseif t < 160
    % Straight line left from [4,7] to [0,7]
    eta_f = [-0.2*(t - 140) + 4; 7; 0];
    eta_f_dot = [-0.2; 0; 0];
    eta_f_ddot = [0; 0; 0];

     eta_d = [4; 9; 0];
        eta_d_dot = [0; 0; 0];
        eta_d_ddot = [0; 0; 0];

elseif t < 180
    % Circular arc from [0,7] to [0,9]
    eta_f = [-sin(0.05*pi*(t - 160)); -cos(0.05*pi*(t - 160)) + 8; 0];
    eta_f_dot = [-0.05*pi*cos(0.05*pi*(t - 160)); 0.05*pi*sin(0.05*pi*(t - 160)); 0];
    eta_f_ddot = [(0.05*pi)^2*sin(0.05*pi*(t - 160)); (0.05*pi)^2*cos(0.05*pi*(t - 160)); 0];

     eta_d = [4; 9; 0];
        eta_d_dot = [0; 0; 0];
        eta_d_ddot = [0; 0; 0];

elseif t <= 200
    % Final straight line from [0,9] to [4,9]
    eta_f = [0.2*(t - 180); 9; 0];
    eta_f_dot = [0.2; 0; 0];
    eta_f_ddot = [0; 0; 0];

     eta_d = [4; 9; 0];
        eta_d_dot = [0; 0; 0];
        eta_d_ddot = [0; 0; 0];

else
    % Final hold
    eta_f = [4; 9; 0];
    eta_f_dot = [0; 0; 0];
    eta_f_ddot = [0; 0; 0];

     eta_d = [4; 9; 0];
     eta_d_dot = [0; 0; 0];
     eta_d_ddot = [0; 0; 0];
end


   % Desired trajectory
    x_d = eta_d(1);
    y_d = eta_d(2);
    x_f = eta_f(1);
    y_f = eta_f(2);
    
    dx_d = eta_d_dot(1);
    dy_d = eta_d_dot(2);
    ddx_d =eta_d_ddot(1);
    ddy_d = eta_d_ddot(2);
    theta_d = atan2(dy_d,dx_d);
    v_d_current = sqrt(dx_d^2 + dy_d^2);
    omega_d_current = (ddy_d * dx_d - ddx_d * dy_d) / (dx_d^2 + dy_d^2 + 1e-6); % approximate

 % Compute control inputs
    [v, omega] = unicycle_tracking_control(x, y, theta, x_d, y_d, theta_d, v_d_current, omega_d_current, k1, k2, k3);
    
    % Robot kinematics update
    x = x + dt * v * cos(theta);
    y = y + dt * v * sin(theta);
    theta = theta + dt * omega;
%  theta = wrapTo2Pi(theta);

    % Log data
    X(i) = x; Y(i) = y; THETA(i) = theta;

    X_d(i) = x_d; Y_d(i) = y_d;
    X_f(i) = x_f; Y_f(i) = y_f;
    phi(i) = atan(l*omega/v);
end


% Plotting
figure;

plot(X, Y, 'b-', 'LineWidth', 3);hold on;
plot(X_f, Y_f, 'r--', 'LineWidth', 2); 

legend('Desired', 'Actual');
xlabel('x'); ylabel('y');
title('Unicycle Trajectory Tracking');
axis equal; grid on;

veh_box = [-b,b,b,l,l,b,b,-b,-b;-D,-D,-d,-d,d,d,D,D,-D];
wheel_s = [-a2,a2,a2,-a2,-a2;-t2/2,-t2/2,t2/2,t2/2,-t2/2];
wheel_b = [-a1,a1,a1,-a1,-a1;-t1/2,-t1/2,t1/2,t1/2,-t1/2];

for i = 1:50:length(X)
     elapsed_time = time(i);  % Get elapsed time in seconds
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
    plot(X_f,Y_f,'k','LineWidth',1)
    plot(X(1:i),Y(1:i),'r--','LineWidth',1)
   
    title(sprintf('Time elapsed: %.2f seconds', elapsed_time),'Interpreter','latex','FontSize',12);
    set(gca, 'fontsize', 14)
    set(gca, 'TickLabelInterpreter','latex');
    set(gca, 'FontSize', 16); set(gcf,'color','w'); grid on; box on;
    
    xmin = min(X) - 0.5;
    xmax = max(X) + 0.5;
    ymin = min(Y) - 0.5;
    ymax = max(Y) + 0.5;
    axis([-3.2 7.2 -1 10])
    %axis equal
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
  v = v_d * cos(e_theta) + k1 * e_x;
%    v=v_d;
    omega = omega_d + k2 * e_y + k3 * sin(e_theta);
end

