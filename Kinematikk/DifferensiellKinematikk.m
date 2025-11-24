%% Fasadevask-Arm – Differensiell kinematikk


clear; clc; close all;

%% --- Parametre og startposisjon ---------------------------------------

% Joint-vinkler i grader  [yaw1 pitch2 pitch3 headPitch headYaw]
qdeg0 = [0 0 0 0 0];

% Lengder
L1    = 3;    % base -> skulder
L2    = 3;    % skulder -> albue
L3    = 2;    % albue -> "hode"
Ltool = 2;    % verktøy-offset langs verktøy-z
a2    = 0.8;  % offset skulderledd
a3    = 0.8;  % offset albueledd

% Konverter til radianer (rad-vektor)
q0 = deg2rad(qdeg0(:)');   % rad-vektor (1x5)

%% --- Bygg manipulator med DH (standard form) --------------------------

L(1) = Link([0  L1  0   pi/2  0 0], 'standard');   % 1: Base yaw
L(2) = Link([0  a2  L2     0  0 0], 'standard');   % 2: Skulder pitch
L(3) = Link([0  a3  L3     0  0 0], 'standard');   % 3: Albue pitch
L(4) = Link([0  0   0   pi/2  0 0], 'standard');   % 4: Hode pitch
L(5) = Link([0  0   0   pi/2  0 0], 'standard');   % 5: Hode yaw

arm = SerialLink(L, 'name', 'Fasadevask-Arm');

% Verktøy-transform (f.eks. spylerør langs z-aksen)
T_tool   = transl(0, 0, Ltool);
arm.tool = T_tool;

%% --- Startpose: fremoverkinematikk og plot ----------------------------

q   = q0;                 % nåværende leddvinkler (rad-vektor)
T0T = arm.fkine(q);       % pose verktøy i base-ramme (SE3)

disp('Startpose (T0T):');
disp(T0T.T);              % skriv ut 4x4-matrisa

figure;
arm.plot(q, 'workspace', [-8 8 -8 8 0 10]);
title('Fasadevask-Arm – startpose');
drawnow;

% Differensiell kinematikk: xdot = J0(q) * qdot
% Bruker pseudoinvers:      qdot = pinv(J0(q)) * xdot_d

Tend = 1.0;      % total simuleringstid [s]
dt   = 0.01;     % tidssteg [s]

q = q0;          % reset til startposisjon

for t = 0:dt:Tend
    
    % Fremoverkinematikk og Jacobian i base-ramme {0}
    T0T = arm.fkine(q);
    J0  = arm.jacob0(q);  % 6x5 matrise
    
    % Ønsket kartesisk hastighet (base-ramme):
    % [vx; vy; vz; wx; wy; wz]
    xdot_0 = [0; 0.05; 0; 0; 0; 0];   % 5 cm/s langs +y
    
    % Leddhastigheter via pseudoinvers (inverse differensiell kinematikk)
    qdot = pinv(J0) * xdot_0;         % 5x1
    
    % Diskret integrasjon (Euler) av leddvinkler
    q = q + (qdot.' * dt);            % q som rad-vektor (1x5)
    
    % Plot bevegelsen
    arm.plot(q);
    title(sprintf('Differensiell kinematikk – t = %.2f s', t));
    drawnow;
end
