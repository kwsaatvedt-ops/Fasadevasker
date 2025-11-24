clear; clc; close all;

%% --- Parametre for vegg og ønsket avstand ---
d_ref   = 2.0;    % ønsket avstand til vegg [m]
x_vegg  = 0.0;    % veggen ligger langs y-aksen (x = 0)

%% --- P-regulator-parametre ---
Kp      = 0.01;    % P-forsterkning (juster for å se effekt)
v_frem  = 0.5;    % konstant fremoverhastighet [m/s]

%% --- Simulasjonsoppsett ---
dt      = 0.05;   % tidssteg [s]
T_slutt = 40;     % simuleringstid [s]
N       = round(T_slutt/dt);

t       = linspace(0, T_slutt, N);

% Tilstandsvektor: [x; y; theta]
x     = zeros(1, N);
y     = zeros(1, N);
theta = zeros(1, N);

% Initialtilstand: start litt for langt fra veggen
x(1)     = 3.0;        % [m]
y(1)     = 0.0;        % [m]
theta(1) = pi/2;       % [rad] peker oppover langs +y

omega   = zeros(1, N); % lagrer styrehastighet
feil    = zeros(1, N); % lagrer feil (x - d_ref)

%% --- Simuler unicycle-modellen med P-kontroll ---
for k = 1:N-1

    % Avstand til vegg i dette enkle oppsettet er bare x-posisjonen
    d_k = x(k);

    % Kontrollfeil: positiv dersom vi er for langt fra veggen (x > d_ref)
    feil(k) = d_k - d_ref;

    % P-regulator:
    omega(k) = Kp * feil(k);

    % Kinematikk for enkel unicycle-modell:
    %   x_dot     = v * cos(theta)
    %   y_dot     = v * sin(theta)
    %   theta_dot = omega
    x_dot     = v_frem * cos(theta(k));
    y_dot     = v_frem * sin(theta(k));
    theta_dot = omega(k);

    % Diskret integrasjon (Euler fremover)
    x(k+1)     = x(k)     + x_dot     * dt;
    y(k+1)     = y(k)     + y_dot     * dt;
    theta(k+1) = theta(k) + theta_dot * dt;
end

% Siste punkt (bare for å ha noe å plotte)
feil(end)  = x(end) - d_ref;
omega(end) = omega(end-1);

%% --- Plot 1: Bane i planet + vegg ---
figure(1); clf; hold on; grid on; axis equal;
title('Mobil robot med P-regulator for avstand til vegg');
xlabel('x [m]');
ylabel('y [m]');

% Plott veggen (x = 0) og ønsket linje (x = d_ref)
y_aksen = linspace(min(y)-1, max(y)+1, 2);
plot(x_vegg * ones(size(y_aksen)), y_aksen, 'k--', 'LineWidth', 1.5);
plot(d_ref  * ones(size(y_aksen)), y_aksen, 'g-.', 'LineWidth', 1.5);

% Plott banen til roboten
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'bo', 'MarkerFaceColor', 'b');              % start
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');          % slutt
legend({'Vegg (x = 0)', 'Ønsket avstand (x = d_{ref})', ...
        'Robotbane', 'Start', 'Slutt'}, ...
        'Location', 'best');

%% --- Plot 2: Avstand til vegg som funksjon av tid ---
figure(2); clf; hold on; grid on;
title('Avstand til vegg og kontrollsignal (P-regulator)');
xlabel('Tid [s]');

d = x;  % avstand til vegg i dette enkle tilfellet

yyaxis left
plot(t, d, 'b', 'LineWidth', 2);
yline(d_ref, 'g--', 'LineWidth', 1.5);
ylabel('Avstand til vegg d(t) [m]');

yyaxis right
plot(t, omega, 'r', 'LineWidth', 1.5);
ylabel('\omega(t) [rad/s]');

legend({'d(t)', 'd_{ref}', '\omega(t)'}, 'Location', 'best');
