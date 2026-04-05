%-----Please extract the compressed file in a  folder and then run----%
clc;clear; close all;
%% solver details
dt = 0.05;
ts = 160;
t = 0:dt:ts;
%% Tractor details
 a1 = 0.15; a2 = 0.08;
 t1 = 0.06; t2 = 0.04;
 D = 0.2; d = 0.1; l = 0.5; b = 0.10;
% Load trajectory
load('DataXY.mat');       % Load precomputed reference trajectory from file
xd = DataXY(:,1);         % Extract x-coordinates of desired path
yd = DataXY(:,2);         % Extract y-coordinates of desired path
trajectory = [xd, yd];    % Combine into [x y] pairs for convenience
N = length(xd);           % Total number of trajectory points

% Parameters
dt = 0.1;               % Time step (s)
v = 0.5;                  % Constant linear velocity (m/s)-------------------->  Requirement mentioned
k_theta = 3;              % Proportional gain for heading error control
lookahead = 10;           % Number of points ahead of the closest point to track
max_steps = 20000;        % Maximum number of iterations for simulation
final_tolerance = 0.05;       % Position tolerance for stopping (m)
heading_tolerance = deg2rad(5); % Heading angle tolerance in radians (~5 degrees)

% Initial condition
x0 = 0; y0 = -2; psi0 = pi/4; % Initial position (x, y) and orientation (psi)
eta = zeros(3, max_steps);   % Preallocate state matrix: [x; y; psi]
eta(:,1) = [x0; y0; psi0];   % Assign initial state

step = 1;        % Start step counter
done = false;    % Flag to indicate completion

% Loop until done or max steps reached
while ~done && step < max_steps
    x = eta(1,step);    % Current x-position
    y = eta(2,step);    % Current y-position
    psi = eta(3,step);  % Current orientation

    % Compute distance to final goal
    dx_end = xd(end) - x;                                      % Difference in x from goal
    dy_end = yd(end) - y;                                      % Difference in y from goal
    dist_to_goal = hypot(dx_end, dy_end);          % Euclidean distance to goal
    heading_error_to_goal = wrapToPi(atan2(dy_end, dx_end) - psi); % Heading error to goal

    % Check stopping condition (close enough in both position and heading)
    if dist_to_goal < final_tolerance && abs(heading_error_to_goal) < heading_tolerance
        done = true;    % If within tolerances, stop simulation
        break;
    end

    % Find closest point on the trajectory to current position
    distances = vecnorm(trajectory - [x y], 2, 2);      % Euclidean distance to all points
    [~, idx_closest] = min(distances);                        % Index of closest point
    idx_target = min(idx_closest + lookahead, N);      % Lookahead target index, clamped to end

    % Target point to track
    x_t = xd(idx_target); % Target x
    y_t = yd(idx_target); % Target y

    % Compute control
    theta_d = atan2(y_t - y, x_t - x);    % Desired heading to target (tangential direction of waypoint)
    e_theta = wrapToPi(theta_d - psi);   % Heading error
    w = k_theta * e_theta;               % Only Control input is angular velocity-------------------------------->
 
    % Kinematic model integration (Euler method)
    eta_dot = [v * cos(psi);   % x dot
               v * sin(psi);   % y dot
               w];             % psi dot
    
    % Update robot state
    eta(:,step+1) = eta(:,step) + eta_dot * dt; % Discrete-time integration
    time(i,1)=step;
    step = step + 1;                            % Advance step counter
    phi(i) = atan(l*w/v);
end

% Trim unused columns (after final step) To stop robot once it reach end
 eta = eta(:,1:step);

veh_box = [-b,b,b,l,l,b,b,-b,-b;-D,-D,-d,-d,d,d,D,D,-D];
wheel_s = [-a2,a2,a2,-a2,-a2;-t2/2,-t2/2,t2/2,t2/2,-t2/2];
wheel_b = [-a1,a1,a1,-a1,-a1;-t1/2,-t1/2,t1/2,t1/2,-t1/2];

for i = 1:5:length(t)
    psi = eta(3,i); 
    R = [cos(psi),-sin(psi);sin(psi),cos(psi)];
    v_m = R*veh_box;
    w_m3 = R*(wheel_b+[0;D+t1/2]);
    w_m4 = R*(wheel_b+[0;-D-t1/2]);
    R1 = [cos(phi(i)),-sin(phi(i));sin(phi(i)),cos(phi(i))];

    w_m1 = R*(R1*wheel_s+[l;d+t2/2]);
    w_m2 = R*(R1*wheel_s+[l;-d-t2/2]);
    fill(v_m(1,:)+x(i),v_m(2,:)+y(i),'y');
    hold on
    fill(w_m1(1,:)+x(i),w_m1(2,:)+y(i),'r');
    fill(w_m2(1,:)+x(i),w_m2(2,:)+y(i),'r');
    fill(w_m3(1,:)+x(i),w_m3(2,:)+y(i),'g');
    fill(w_m4(1,:)+x(i),w_m4(2,:)+y(i),'g');
%     plot(xr,yr,'m--')
    plot(eta(1,1:i),eta(2,1:i),'b-')
    
    xmin = min(eta(1,:)) - 0.5;
    xmax = max(eta(1,:)) + 0.5;
    ymin = min(eta(2,:)) - 0.5;
    ymax = max(eta(2,:)) + 0.5;
    axis([xmin xmax ymin ymax])
    axis equal
    grid on
    xlabel('x,[m]')
    ylabel('y,[m]');
    pause(0.01)
    hold off
end

%%
%  %% Animation of Realistic Tractor
% 
% % Main body dimensions
% l = 2.4;       % length of tractor base
% w = 1.2;       % width of tractor base
% 
% % Cabin dimensions
% cab_l = 0.8;   
% cab_w = 0.9;
% cab_x_offset = -0.3; % relative to body center
% 
% % Hood
% hood_l = 0.5;
% hood_w = 0.6;
% hood_x_offset = l/2 - hood_l/2;
% 
% % Rear Wheels
% rear_wheel_r = 0.5; % radius
% rear_wheel_w = 0.3; % width
% 
% % Front Wheels
% front_wheel_r = 0.25;
% front_wheel_w = 0.2;
% 
% % Rear Wheel Positions
% rear_wheel_pos = [-l/2 + 0.2,  w/2 + rear_wheel_w/2;
%                   -l/2 + 0.2, -w/2 - rear_wheel_w/2];
% 
% % Front Wheel Positions (projecting slightly outside)
% front_wheel_pos = [l/2 - 0.3,  l/2 - 0.3;
%                    w/2 + 0.1, -w/2 - 0.1];
% 
% figure(1)
% tic
% for i = 1:5:size(eta, 2)
%     psi = eta(3,i);
%     R_psi = [cos(psi), -sin(psi);
%              sin(psi),  cos(psi)];
%     x = eta(1,i);
%     y = eta(2,i);
% 
%     %% Body
%     body_shape = [-l/2, l/2, l/2, -l/2, -l/2;
%                   -w/2, -w/2, w/2, w/2, -w/2];
%     body_pos = R_psi * body_shape;
%     fill(body_pos(1,:) + x, body_pos(2,:) + y, [0.3 0.6 0.9]); hold on; % blue base
% 
%     %% Cabin
%     cabin_shape = [-cab_l/2, cab_l/2, cab_l/2, -cab_l/2, -cab_l/2;
%                    -cab_w/2, -cab_w/2, cab_w/2, cab_w/2, -cab_w/2];
%     cabin_center = R_psi * [cab_x_offset; 0];
%     cabin_pos = R_psi * cabin_shape;
%     fill(cabin_pos(1,:) + x + cabin_center(1), cabin_pos(2,:) + y + cabin_center(2), [0.8 0.8 0.2]); % yellow cabin
% 
%     %% Hood
%     hood_shape = [-hood_l/2, hood_l/2, hood_l/2, -hood_l/2, -hood_l/2;
%                   -hood_w/2, -hood_w/2, hood_w/2, hood_w/2, -hood_w/2];
%     hood_center = R_psi * [hood_x_offset; 0];
%     hood_pos = R_psi * hood_shape;
%     fill(hood_pos(1,:) + x + hood_center(1), hood_pos(2,:) + y + hood_center(2), [0.5 0.5 0.5]); % gray hood
% 
%     %% Rear Wheels
%     for k = 1:2
%         pos = rear_wheel_pos(k,:)';
%         wheel_center = R_psi * pos;
%         theta = linspace(0, 2*pi, 30);
%         fill(rear_wheel_r*cos(theta) + x + wheel_center(1), ...
%              rear_wheel_r*sin(theta) + y + wheel_center(2), 'k');
%     end
% 
%     %% Front Wheels
%     for k = 1:2
%         pos = front_wheel_pos(:,k);
%         wheel_center = R_psi * pos;
%         theta = linspace(0, 2*pi, 30);
%         fill(front_wheel_r*cos(theta) + x + wheel_center(1), ...
%              front_wheel_r*sin(theta) + y + wheel_center(2), [0.2 0.2 0.2]);
%     end
% 
%     %% Heading arrow
%     heading_length = 2.8;
%     quiver(x, y, heading_length*cos(psi), heading_length*sin(psi), 'm', 'LineWidth', 3, 'MaxHeadSize', 2)
% 
%     %% Path and reference
%     plot(eta(1,1:i), eta(2,1:i), 'b-', 'LineWidth', 2);
%     plot(xd, yd, 'r--', 'LineWidth', 1.5); hold on;
% 
% %     title('Magenta arrow-Heading direction','Interpreter','latex','FontSize',12);
%     elapsed_time = toc;  % Get elapsed time in seconds
%     title(sprintf('Time elapsed: %.2f seconds', elapsed_time),'Interpreter','latex','FontSize',12);
%     set(gca, 'fontsize', 14)
%     set(gca, 'TickLabelInterpreter','latex');
%     set(gca, 'FontSize', 16); set(gcf,'color','w'); grid on; box on;
%     xlabel('$x~[m]$', 'Interpreter', 'latex', 'FontSize', 16);
%     ylabel('$y~[m]$', 'Interpreter', 'latex', 'FontSize', 16);
%     axis([-3.5 24 -4 15]); axis square
%     pause(0.001); hold off;
% end