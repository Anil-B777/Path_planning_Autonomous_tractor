clc; clear; close all;
% Tractor details
a1 = 0.15; a2 = 0.08;
t1 = 0.06; t2 = 0.04;
D = 4*0.2; d = 4*0.1; b = 4*0.10;

%% Parameters
L = 6;                      % Wheelbase (meters)
l=L;
v = 8.0;                    % Constant forward speed (m/s)
dt = 0.05;                  % Simulation time step (seconds)
lookahead = 2.0;            % Lookahead distance for pure pursuit (meters)
swath_width = 6;            % Width between swaths (meters)
num_swaths = 9;             % Number of swaths to cover
field_length = 40;          % Length of the field (meters)
turn_radius = swath_width/8;  % Radius for U-turn arcs (meters)

%% Generate path: Connect only odd swaths with alternating directions and arcs on correct sides
path = [];

odd_swaths = 1:2:num_swaths; % [1 3 5 7 9]

for idx = 1:length(odd_swaths)
    i = odd_swaths(idx);
    y_swath = (i - 1) * swath_width;
    
    % Alternate direction for odd swaths:
    % odd idx (1st, 3rd...) go left-to-right
    % even idx (2nd, 4th...) go right-to-left
    if mod(idx, 2) == 1
        x_line = linspace(0, field_length, 100);
    else
        x_line = linspace(field_length, 0, 100);
    end
    y_line = y_swath * ones(size(x_line));
    path = [path [x_line; y_line]];
    
    % Add arc turn to next odd swath if not last odd swath
    if idx < length(odd_swaths)
        next_i = odd_swaths(idx + 1);
        next_y = (next_i - 1) * swath_width;
        
        % Determine arc center and angle depending on current swath direction
        if mod(idx, 2) == 1
            % Current swath left-to-right => arc outside RIGHT boundary
            cx = field_length + turn_radius/2;
            cy = (y_swath + next_y) / 2;
            theta = linspace(-pi/2, pi/2, 100);  % CCW arc from bottom to top
        else
            % Current swath right-to-left => arc outside LEFT boundary
            cx = -turn_radius/2;
            cy = (y_swath + next_y) / 2;
            theta = linspace(3*pi/2, pi/2, 100);  % CW arc from top to bottom
        end
        
        x_arc = cx + turn_radius * cos(theta);
        y_arc = cy + turn_radius * sin(theta);
        path = [path [x_arc; y_arc]];
    end
end

%% Initial tractor state (start at beginning of swath 1)
x = 0;
y = 0;
theta = 0;  % Facing along positive x-axis
traj = [x; y];

idx = 1;  % Index of current lookahead target point on path
THETA = zeros(size(path, 2), 1);
phi = zeros(size(path, 2), 1);

%% Simulation loop
count = 1;
while true
    if idx >= size(path, 2)
        v = 0;  % Finished path
        break;
    end
    
    % Find pursuit target point on path
    while idx < size(path, 2)
        dx = path(1, idx) - x;
        dy = path(2, idx) - y;
        if hypot(dx, dy) > lookahead
            break;
        end
        idx = idx + 1;
    end
    
    target = path(:, idx);
    dx = target(1) - x;
    dy = target(2) - y;
    
    % steering angle calculation using pure pursuit
    local_x = cos(theta) * dx + sin(theta) * dy;
    local_y = -sin(theta) * dx + cos(theta) * dy;
    alpha = atan2(local_y, local_x);
    steer = atan2(2 * L * sin(alpha) / lookahead, 1);
    
    % Bicycle model kinematics update
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + v / L * tan(steer) * dt;
    
    THETA(count) = theta;
    phi(count) = steer;
    X(count) = x;
    Y(count) = y;
    traj = [traj [x; y]];
    
    % Plot field boundary and path
    clf; hold on; axis equal;
    plot([0, field_length, field_length, 0, 0], ...
         [0, 0, swath_width * (num_swaths - 1), swath_width * (num_swaths - 1), 0], ...
         'g-', 'LineWidth', 2);
    plot(path(1, :), path(2, :), 'k--', 'LineWidth', 1);   % Planned path
    
    % Plot tractor trajectory so far
    plot(traj(1, :), traj(2, :), 'b-', 'LineWidth', 2);
    
    % Draw tractor position as a red circle
    plot(x, y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    for i = 1:num_swaths
    y_swath = (i - 1) * swath_width;
    
    if mod(i, 2) == 1
        x_line = [0, field_length];
    else
        x_line = [field_length, 0];
    end
    
    plot(x_line, [y_swath, y_swath], 'r--', 'LineWidth', 1);
end
    % Display current time
    title(sprintf('Time: %.1f seconds', dt * (size(traj, 2) - 1)));
    
    % Axis limits with margin
    margin = 5;
    xlim([-turn_radius - margin, field_length + turn_radius + margin]);
    ylim([-margin, swath_width * num_swaths + margin]);
    
    drawnow;
    count = count + 1;
end

%% Animation
plot([0, field_length, field_length, 0, 0],[0, 0, swath_width * (num_swaths - 1), swath_width * (num_swaths - 1), 0],'g-', 'LineWidth', 2);
hold on;

veh_box = [-b,b,b,l,l,b,b,-b,-b; -D,-D,-d,-d,d,d,D,D,-D];
wheel_s = [-a2,a2,a2,-a2,-a2; -t2/2,-t2/2,t2/2,t2/2,-t2/2];
wheel_b = [-a1,a1,a1,-a1,-a1; -t1/2,-t1/2,t1/2,t1/2,-t1/2];

for i = 1:7:length(X)
    psi = THETA(i);
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    v_m = R * veh_box;
    w_m3 = R * (wheel_b + [0; D + t1/2]);
    w_m4 = R * (wheel_b + [0; -D - t1/2]);
    R1 = [cos(phi(i)), -sin(phi(i)); sin(phi(i)), cos(phi(i))];
    w_m1 = R * (R1 * wheel_s + [l; d + t2/2]);
    w_m2 = R * (R1 * wheel_s + [l; -d - t2/2]);
    
    fill(v_m(1, :) + X(i), v_m(2, :) + Y(i), 'y');
    hold on;
    fill(w_m1(1, :) + X(i), w_m1(2, :) + Y(i), 'r');
    fill(w_m2(1, :) + X(i), w_m2(2, :) + Y(i), 'r');
    fill(w_m3(1, :) + X(i), w_m3(2, :) + Y(i), 'g');
    fill(w_m4(1, :) + X(i), w_m4(2, :) + Y(i), 'g');
    plot(X(1:i), Y(1:i), 'r', 'LineWidth', 2);
    
    set(gca, 'fontsize', 14);
    set(gca, 'TickLabelInterpreter', 'latex');
    set(gca, 'FontSize', 16);
    set(gcf, 'color', 'w');
    grid on; box on;
    
    xmin = min(X) - 5;
    xmax = max(X) + 5;
    ymin = min(Y) - 5;
    ymax = max(Y) + 7;
    axis([xmin xmax ymin ymax]);
    grid on;
    xlabel('x, [m]');
    ylabel('y, [m]');
    pause(0.01);
    hold on;
end
