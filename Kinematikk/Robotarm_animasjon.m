function demo_facade_snake()
    %% --- 1. Lag arm-modellen (fra ditt script) ---
    % Startkonfig (grader) – brukes bare som initial gjetning
    q0deg = [0 30 20 10 15];   % [yaw1 pitch2 pitch3 headPitch headYaw]
    [~, arm] = facade_arm(q0deg, false);   % useToolbox=false -> ingen teach her
    q_init = deg2rad(q0deg(:)');           % 1x5 radvektor

    %% --- 2. Definer "vegg" og slange-striper ---
    % Veggen: vi holder X fast, og flytter oss i Y (sideveis) og Z (høyde)
    x_wall   = 7;       % avstand til vegg (juster om IK feiler)
    z_bottom = 2.0;     % nederst på fasaden
    z_top    = 7.0;     % geometrisk topp på fasaden

    dz_top   = 3;     % liten margin under topp for å unngå IK-flip

    n_stripes         = 5;   % antall vertikale striper i slangen
    points_per_stripe = 40;  % punkter per vertikal stripe
    points_side       = 15;  % punkter for sideveis-bevegelse

    % Y-posisjonene til hver stripe (sideveis forskyvning)
    y_center = 0;
    y_step   = 0.7;                          % hvor mye "litt til siden" betyr
    y_list   = y_center + y_step * ((0:n_stripes-1) - (n_stripes-1)/2);

    % Vi bryr oss bare om posisjon i IK → enklere å konvergere
    mask = [1 1 1 0 0 0];

    %% --- 3. Bygg opp hele slange-banen i leddrommet ---
    Qtraj = [];   % hit samler vi alle leddvinklene

    for s = 1:n_stripes
        y = y_list(s);

        % Oddetalls-striper: bunn -> "topp-nivå"
        % Partalls-striper: "topp-nivå" -> bunn
        if mod(s,2) == 1
            z_end = z_top - dz_top;  % litt under topp
            z_vec = linspace(z_bottom, z_end, points_per_stripe);
        else
            z_end = z_bottom;        % helt ned
            z_vec = linspace(z_top - dz_top, z_bottom, points_per_stripe);
        end

        % --- Vertikal stripe ---
        for i = 1:length(z_vec)
            z = z_vec(i);

            % Ønsket pose på veggen (kun posisjon viktig i IK)
            T_des = transl(x_wall, y, z);

            % Invers kinematikk med forrige løsning som start
            q_sol = arm.ikine(T_des, q_init, 'mask', mask);

            % Sørg for radvektor
            q_sol = q_sol(:)';   

            % Legg til i trajektorien
            Qtraj = [Qtraj; q_sol]; %#ok<AGROW>

            % Neste punkt starter fra forrige løsning
            q_init = q_sol;
        end

        % --- Sideveis bevegelse på topp/bunn (hvis ikke siste stripe) ---
        if s < n_stripes
            y_next = y_list(s+1);

            % Sluttposisjon for sidebevegelse i kartesisk rom
            T_side_end = transl(x_wall, y_next, z_end);

            % Finn leddkonfigurasjon for sluttpunktet, start fra q_init
            q_side_goal = arm.ikine(T_side_end, q_init, 'mask', mask);
            q_side_goal = q_side_goal(:)';   % radvektor

            % Lag jevn bevegelse i leddrom mellom q_init og q_side_goal
            [q_side_traj, ~] = jtraj(q_init, q_side_goal, points_side);

            % Hopp over første punkt (samme som siste punkt på stripa)
            q_side_traj = q_side_traj(2:end, :);

            % Legg til i totaltrajektorien
            Qtraj = [Qtraj; q_side_traj]; %#ok<AGROW>

            % Oppdater q_init til sluttposisjonen for neste stripe
            q_init = q_side_goal;
        end
    end

    %% --- 4. Tegn en enkel vegg ---
    figure('Name','Fasadevask – slangebane'); clf;
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45, 25);

    % Veggflate bare for visuelt inntrykk
    [Yw, Zw] = meshgrid(-4:1:4, 0:1:10);
    Xw = x_wall * ones(size(Yw));
    surf(Xw, Yw, Zw, 'FaceAlpha', 0.1, 'EdgeColor', 'none');

    title('Fasadevaskrobot – slange-mønster med horisontale topp/bunn-segment');

    %% --- 5. Animer armen langs slange-trajektorien ---
    for k = 1:size(Qtraj,1)
        arm.plot(Qtraj(k,:), ...
            'workspace', [-1 10 -6 6 0 10], ...
            'view', [45 25], ...
            'nowrist', ...
            'delay', 0.01);
        drawnow;
    end
end
