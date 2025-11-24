function [T06, arm] = facade_arm(qdeg, useToolbox)
    if nargin < 1, qdeg = [0 30 20 10 15]; end   % [yaw1 pitch2 pitch3 headPitch headYaw]
    if nargin < 2, useToolbox = true; end

    % Lengder
    L1 = 3; L2 = 3; L3 = 2; Ltool = 2;
    a2 = 0.8;
    a3 = 0.8; % skulderoffset (m)
    q = deg2rad(qdeg(:)');

    %% --- DH-definisjon (standard form)
    L(1) = Link([0  L1  0   pi/2  0 0], 'standard');   % 1: Base yaw
    L(2) = Link([0  a2  L2     0  0 0], 'standard');   % 2: Skulder pitch
    L(3) = Link([0  a3  L3     0  0 0], 'standard');   % 3: Albue pitch
    L(4) = Link([0  0   0   pi/2  0 0], 'standard');   % 4: Hode pitch
    L(5) = Link([0  0   0   pi/2  0 0], 'standard');   % 5: Hode pitch

    arm = SerialLink(L, 'name', 'Fasadevask-Arm');

    % Tool: 2 m ut i lokal +X
    arm.tool = transl(Ltool, 0, 0);

    % Base
    arm.base = eye(4);

    % Bare for konsistens, returner T06 også:
    T06 = arm.fkine(q);

    %% --- Visualisering (valgfri teach)
    if useToolbox && exist('SerialLink','class')
        figure('Name','Fasadevaskrobot – yaw/pitch'); clf;
        arm.plot(q,'workspace',[-8 8 -8 8 0 10],'view',[60 30],'nowrist');
        hold on; grid on; axis equal;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Fasadevaskrobot – Yaw-base, Pitch-ledd og offset skulder');
    end
end
