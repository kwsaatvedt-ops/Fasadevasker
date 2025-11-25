%% motion_planning_demo.m
% Robot cleaning pattern: tett, snake-aktig opp/ned + sidelengs

clear; clc; close all;

%% 1) Define robot (same as earlier)
L1 = 3; L2 = 3; L3 = 2; a2 = 0.8; a3 = 0.8; Ltool = 2;

L(1) = Link([0  L1  0   pi/2  0  0], 'standard');
L(2) = Link([0  a2  L2  0     0  0], 'standard');
L(3) = Link([0  a3  L3  0     0  0], 'standard');
L(4) = Link([0  0   0   pi/2  0  0], 'standard');
L(5) = Link([0  0   0   pi/2  0  0], 'standard');

arm = SerialLink(L, 'name', '5DOF_arm');
arm.tool = transl(0,0,Ltool);

mask = [1 1 1 0 0 0];

%% 2) Define cleaning area (coordinates in base frame)
x_start = 6;    % distance from base to facade
y_start = -2;   % left side
y_end   =  2;   % right side
z_top   =  5;   % top position
z_bottom=  1;   % bottom position

% Flere striper -> tettere sidelengs dekning
num_stripes = 10;                       % øk for tettere vask
stripe_step = (y_end - y_start) / (num_stripes - 1);

%% 3) Plan cleaning pattern: one vertical pass per stripe, then sideways
q_traj = [];

current_y = y_start;

% Litt saktere: flere punkter i trajektoriene
Nvert  = 80;   % vertikal bevegelse (opp/ned)
Nhoriz = 40;   % sidelengs bevegelse

for i = 1:num_stripes
    
    % Bestem om vi går ned eller opp på denne stripen
    if mod(i,2) == 1
        % Odd stripe: topp -> bunn, flytt sidelengs på bunn
        T_start = transl(x_start, current_y, z_top);
        T_end   = transl(x_start, current_y, z_bottom);
        z_side  = z_bottom;
    else
        % Partall stripe: bunn -> topp, flytt sidelengs på topp
        T_start = transl(x_start, current_y, z_bottom);
        T_end   = transl(x_start, current_y, z_top);
        z_side  = z_top;
    end
    
    % IK for start og slutt på vertikal bevegelse
    q_start = arm.ikine(T_start, 'mask', mask);
    q_end   = arm.ikine(T_end,   'mask', mask);
    
    % Vertikal bevegelse (én vei: opp eller ned)
    [qv, ~, ~] = jtraj(q_start, q_end, Nvert);
    q_traj = [q_traj; qv];
    
    % Sidelengs til neste stripe (kun hvis det finnes en neste)
    if i < num_stripes
        next_y = current_y + stripe_step;
        y_vec  = linspace(current_y, next_y, Nhoriz);
        
        for j = 1:Nhoriz
            T_side = transl(x_start, y_vec(j), z_side);
            q_side = arm.ikine(T_side, 'mask', mask);
            q_traj = [q_traj; q_side];
        end
        
        current_y = next_y;
    end
end

%% 4) Animate cleaning motion (litt saktere)
figure;
arm.plot(q_traj, 'trail', {'b','LineWidth',2}, 'fps', 15);
title('Tett fasadevask: snake-mønster med mange opp/ned-pass');

%% 5) Plot joint angles
figure;
plot(q_traj);
xlabel('Sample'); ylabel('Joint angle [rad]');
legend('q1','q2','q3','q4','q5');
title('Joint trajectories: dense snake cleaning pattern');
grid on;
