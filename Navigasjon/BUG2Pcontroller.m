clear; clc; close all;

%% --- MÅL, START OG HINDRING ----------------------------------------
% Start- og målposisjon (samme koordinatsystem som i P-kontrollerfiguren)
x_start = 3.0;   y_start = 0.0;
x_goal  = 2.0;   y_goal  = 8.0;

% Hindring: sirkel som ligger på m-line mellom start og mål
obs_cx   = 2.0;   % senter x
obs_cy   = 4.0;   % senter y
obs_R    = 0.6;   % radius [m]
sikkerhet = 0.6;  % sikkerhetsmargin [m]
hit_dist  = obs_R + sikkerhet;  % når vi er nærmere enn dette, er hindring "truffet"

% m-line (rett linje mellom start og mål) brukes flere steder
m_dx = x_goal - x_start;
m_dy = y_goal - y_start;
m_norm = hypot(m_dx, m_dy);        % |m-line vektor|

%% --- KONTROLLER- OG SIMULASJONSPARAMETRE ---------------------------
Kp_theta  = 2.0;    % P-forsterker på retningsfeil
v_frem    = 0.4;    % konstant fremoverhastighet [m/s]

dt        = 0.05;   % tidssteg [s]
T_slutt   = 60;     % maks simuleringstid [s]
N_max     = round(T_slutt/dt);

goal_tol  = 0.15;   % avstandsterskel for å "ha nådd målet" [m]
mline_tol = 0.05;   % hvor nær m-line vi må være for å si "på m-line" [m]

%% --- ARRAYER FOR LAGRING AV TILSTAND -------------------------------
x     = nan(1, N_max);
y     = nan(1, N_max);
theta = nan(1, N_max);
omega = nan(1, N_max);
state_log = strings(1, N_max);  % for å logge hvilken BUG2-tilstand vi er i

% initialtilstand
x(1)     = x_start;
y(1)     = y_start;
theta(1) = pi/2;         % peker oppover langs +y
state_log(1) = "GOAL_STATE";

% Variabler relatert til BUG2 logikk
state = "GOAL_STATE";    % kan være "GOAL_STATE" eller "BOUNDARY_STATE"
hit_point = [NaN; NaN];  % posisjon der vi først traff hindringen
dist_goal_at_hit = NaN;  % avstand til mål ved treffpunkt

%% --- HJELPEFUNKSJON FOR VINKELDIFF -------------------------------
angdiff = @(a,b) atan2(sin(a-b), cos(a-b));   % gir vinkel i [-pi, pi]

%% --- HOVEDLØKKE -----------------------------------------------------
k = 1;
while k < N_max

    % Avstand til mål
    dist_goal = hypot(x_goal - x(k), y_goal - y(k));

    % Stopp om vi er ved målet
    if dist_goal < goal_tol
        fprintf('Mål nådd ved k = %d, tid = %.2f s\n', k, (k-1)*dt);
        break;
    end

    % Avstand til hindring
    dist_obs = hypot(x(k) - obs_cx, y(k) - obs_cy);

    switch state
        case "GOAL_STATE"
            % ---------------------------------------------------------
            % TRINN 1: KJØR RETT MOT MÅL (langs m-line)
            % ---------------------------------------------------------

            % Ønsket retning mot målet
            theta_des = atan2(y_goal - y(k), x_goal - x(k));
            e_theta   = angdiff(theta_des, theta(k));
            omega(k)  = Kp_theta * e_theta;
            v         = v_frem;

            % Sjekk om vi treffer hindringen
            if dist_obs <= hit_dist
                % lagre treffpunkt
                hit_point = [x(k); y(k)];
                dist_goal_at_hit = dist_goal;
                state = "BOUNDARY_STATE";
                fprintf('Bytter til BOUNDARY_STATE ved t = %.2f s\n', (k-1)*dt);
            end

        case "BOUNDARY_STATE"
            % ---------------------------------------------------------
            % TRINN 2: FØLG HINDRINGENS KANT (her: sirkel rundt)
            % ---------------------------------------------------------

            % Vinkel fra hindrings-senter til robot
            phi = atan2(y(k) - obs_cy, x(k) - obs_cx);

            % Tangentretning for å følge hindringen med klokka
            theta_tangent = phi - pi/2;  % CW rundt hindringen

            % Litt P på vinkel mot tangent (holder oss langs sirkelen)
            e_theta  = angdiff(theta_tangent, theta(k));
            omega(k) = Kp_theta * e_theta;
            v        = v_frem;

            % Sjekk om vi bør forlate hindringen (BUG2-kriterium)
            % 1) Vi må være på m-line igjen
            % 2) Vi må være nærmere målet enn da vi traff hindringen
            %    (ellers risikerer vi å gå i sirkel)
            % Avstand fra (x,y) til m-line:
            d_mline = abs(m_dy*(x(k)-x_start) - m_dx*(y(k)-y_start)) / m_norm;

            if d_mline < mline_tol && dist_goal < (dist_goal_at_hit - 1e-3)
                state = "GOAL_STATE";
                fprintf('Forlater hindring og går tilbake til GOAL_STATE ved t = %.2f s\n',...
                        (k-1)*dt);
            end
    end

    % Logg tilstand
    state_log(k) = state;

    % --- UNICYCLE-KINEMATIKK (samme som tidligere skript) ----------
    x_dot     = v * cos(theta(k));
    y_dot     = v * sin(theta(k));
    theta_dot = omega(k);

    x(k+1)     = x(k)     + x_dot     * dt;
    y(k+1)     = y(k)     + y_dot     * dt;
    theta(k+1) = theta(k) + theta_dot * dt;

    k = k + 1;
end

% Trim arrayer til faktisk lengde
x     = x(1:k);
y     = y(1:k);
theta = theta(1:k);
omega = omega(1:k);
state_log = state_log(1:k);
t = (0:k-1) * dt;

%% --- PLOTT: BANE + HINDRING + M-LINE -------------------------------
figure(1); clf; hold on; grid on; axis equal;
title('Mobil robot – BUG2-navigasjon');
xlabel('x [m]'); ylabel('y [m]');

% Vegg som før, hvis du vil ha den med i figuren
x_vegg = 0;
y_aksen = linspace(min(y)-1, max(y)+1, 2);
plot(x_vegg * ones(size(y_aksen)), y_aksen, 'k--', 'LineWidth', 1.5);

% M-line
plot([x_start x_goal], [y_start y_goal], 'g-.', 'LineWidth', 1.5);

% Hindring (sirkel)
th = linspace(0, 2*pi, 100);
plot(obs_cx + obs_R*cos(th), obs_cy + obs_R*sin(th), 'r', 'LineWidth', 2);

% Robotbane
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'bo', 'MarkerFaceColor', 'b');       % start
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');   % slutt

legend({'Vegg (x = 0)', 'bane', ...
        'Hindring', 'Robotbane', 'Start', 'Slutt'}, ...
        'Location', 'best');

%% --- PLOTT: VINKELHASTIGHET OG TILSTAND ----------------------------
figure(2); clf; 
subplot(2,1,1);
plot(t, omega, 'b', 'LineWidth', 1.5); grid on;
xlabel('Tid [s]'); ylabel('\omega(t) [rad/s]');
title('Styresignal fra P-kontroller (retningsfeil)');

subplot(2,1,2);
stairs(t, state_log == "GOAL_STATE", 'LineWidth', 1.5); grid on;
ylim([-0.1 1.1]);
yticks([0 1]); yticklabels({'BOUNDARY','GOAL'});
xlabel('Tid [s]'); ylabel('Tilstand');
title('BUG2-tilstand (1 = GOAL\_STATE, 0 = BOUNDARY\_STATE)');
