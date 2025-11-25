%% Fasadevask-Arm – Script

clear; clc; close all;

% Joint-vinkler i grader  [yaw1 pitch2 pitch3 headPitch headpitch]
qdeg = [0 -20 20 0 0];

% Lengder
L1 = 3; L2 = 3; L3 = 2; Ltool = 2;
a2 = 0.8;
a3 = 0.8; % skulderoffset (m)

% Konverter til radianer
q = deg2rad(qdeg(:)');   % sørger for radianer på rad-vektor

%% --- DH-definisjon (standard form)
L(1) = Link([0  L1  0   pi/2  0 0], 'standard');   % 1: Base yaw
L(2) = Link([0  a2  L2     0  0 0], 'standard');   % 2: Skulder pitch
L(3) = Link([0  a3  L3     0  0 0], 'standard');   % 3: Albue pitch
L(4) = Link([0  0   0   pi/2  0 0], 'standard');   % 4: Hode pitch
L(5) = Link([0  0   0   pi/2  0 0], 'standard');   % 5: Hode yaw

arm = SerialLink(L, 'name', 'Fasadevask-Arm');

%% Fremover kinematikk
T = arm.fkine(q)   % homogent transformasjon

%% Invers kinematikk
qi = arm.ikine(T, "mask", [1 1 1 0 0 0])
Ta = arm.fkine(qi)
J0  = arm.jacob0(q)

figure;
arm.plot(q);
title('(q) Forward Kinematics');
hold on;
