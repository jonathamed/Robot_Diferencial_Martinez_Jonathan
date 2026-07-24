clear; clc; close all;

% 1. Parámetros del robot y simulación
r = 0.06;         % Radio de rueda [m]
L = 0.37;         % Distancia entre ruedas [m]
dt = 0.02;        % Paso de tiempo [s]
T_max = 20;       % Tiempo máximo [s]
t = 0:dt:T_max;

% 2. Parámetros del controlador (según la tabla del reporte)
kp = 0.8;
k_theta = 2.5;
tol = 0.02;       % Tolerancia de llegada [m]
w_max = 12.0;     % theta_dot_max [rad/s]

% 3. Condiciones iniciales y objetivo
x0 = 0; y0 = 0; theta0 = 0;
xg = 2.0; yg = 1.5; thetag = pi/4;  % Ajusta a tu punto objetivo asignado

N = length(t);

%% ========================================================================
%  SIMULACIÓN 1: SIN SATURACIÓN
%  ========================================================================
q_sin = zeros(3, N);
q_sin(:,1) = [x0; y0; theta0];
wD_sin = zeros(1, N); wI_sin = zeros(1, N);
u_sin = zeros(1, N);  w_sin = zeros(1, N);
err_pos_sin = zeros(1, N); err_ang_sin = zeros(1, N);
t_llegada_sin = T_max;

for k = 1:N-1
    x = q_sin(1,k); y = q_sin(2,k); th = q_sin(3,k);
    
    dx = xg - x; dy = yg - y;
    rho = sqrt(dx^2 + dy^2);
    err_pos_sin(k) = rho;
    
    if rho < tol && t_llegada_sin == T_max
        t_llegada_sin = t(k);
    end
    if rho < tol
        q_sin(:, k+1:end) = repmat(q_sin(:,k), 1, N-k);
        break;
    end
    
    alpha = atan2(dy, dx) - th;
    alpha = atan2(sin(alpha), cos(alpha)); % Normalizar
    err_ang_sin(k) = alpha;
    
    % Control
    u = kp * rho;
    w = k_theta * alpha;
    
    % Velocidades de ruedas
    wD = (u + (w * L / 2)) / r;
    wI = (u - (w * L / 2)) / r;
    
    wD_sin(k) = wD; wI_sin(k) = wI;
    u_sin(k) = u;   w_sin(k) = w;
    
    % Integración
    q_sin(1, k+1) = q_sin(1,k) + u * cos(th) * dt;
    q_sin(2, k+1) = q_sin(2,k) + u * sin(th) * dt;
    q_sin(3, k+1) = q_sin(3,k) + w * dt;
end

%% ========================================================================
%  SIMULACIÓN 2: CON SATURACIÓN PROPORCIONAL
%  ========================================================================
q_con = zeros(3, N);
q_con(:,1) = [x0; y0; theta0];
wD_con = zeros(1, N); wI_con = zeros(1, N);
u_con = zeros(1, N);  w_con = zeros(1, N);
err_pos_con = zeros(1, N); err_ang_con = zeros(1, N);
t_llegada_con = T_max;

for k = 1:N-1
    x = q_con(1,k); y = q_con(2,k); th = q_con(3,k);
    
    dx = xg - x; dy = yg - y;
    rho = sqrt(dx^2 + dy^2);
    err_pos_con(k) = rho;
    
    if rho < tol && t_llegada_con == T_max
        t_llegada_con = t(k);
    end
    if rho < tol
        q_con(:, k+1:end) = repmat(q_con(:,k), 1, N-k);
        break;
    end
    
    alpha = atan2(dy, dx) - th;
    alpha = atan2(sin(alpha), cos(alpha));
    err_ang_con(k) = alpha;
    
    % Control sin saturar
    u = kp * rho;
    w = k_theta * alpha;
    
    wD = (u + (w * L / 2)) / r;
    wI = (u - (w * L / 2)) / r;
    
    % --- SATURACIÓN PROPORCIONAL ---
    max_solicitado = max(abs([wD, wI]));
    if max_solicitado > w_max
        factor = w_max / max_solicitado;
        wD = wD * factor;
        wI = wI * factor;
    end
    
    % Recalcular velocidades lineales y angulares reales
    u_sat = (r/2) * (wD + wI);
    w_sat = (r/L) * (wD - wI);
    
    wD_con(k) = wD; wI_con(k) = wI;
    u_con(k) = u_sat; w_con(k) = w_sat;
    
    % Integración
    q_con(1, k+1) = q_con(1,k) + u_sat * cos(th) * dt;
    q_con(2, k+1) = q_con(2,k) + u_sat * sin(th) * dt;
    q_con(3, k+1) = q_con(3,k) + w_sat * dt;
end

%% ========================================================================
%  DESPLIEGUE DE RESULTADOS PARA LA TABLA 8.2
%  ========================================================================
disp('======================================================');
disp('            TABLA 8.2 COMPARACIÓN DE DESEMPEÑO        ');
disp('======================================================');
fprintf('Error final (m):          Sin: %.4f | Con: %.4f\n', err_pos_sin(end-1), err_pos_con(end-1));
fprintf('Tiempo de llegada (s):    Sin: %.2f   | Con: %.2f\n', t_llegada_sin, t_llegada_con);
fprintf('wD maxima (rad/s):        Sin: %.2f  | Con: %.2f\n', max(abs(wD_sin)), max(abs(wD_con)));
fprintf('wI maxima (rad/s):        Sin: %.2f  | Con: %.2f\n', max(abs(wI_sin)), max(abs(wI_con)));

%% ========================================================================
%  GRÁFICA COMPARATIVA (Resultados 8.1 - Ítem 8)
%  ========================================================================
figure(1);
plot(q_sin(1,:), q_sin(2,:), 'r--', 'LineWidth', 2); hold on;
plot(q_con(1,:), q_con(2,:), 'b-', 'LineWidth', 2);
plot(x0, y0, 'go', 'MarkerFaceColor', 'g');
plot(xg, yg, 'ro', 'MarkerFaceColor', 'r');
grid on; axis equal;
title('Comparación de Trayectorias: Sin vs Con Saturación');
xlabel('X [m]'); ylabel('Y [m]');
legend('Sin Saturación', 'Con Saturación Proporcional', 'Inicio', 'Objetivo');

%% ========================================================================
%  GENERACIÓN DE ANIMACIÓN GIF (control_cinematico.gif)
%  ========================================================================
gifFilename = 'control_cinematico.gif';
figAnim = figure(2);
set(figAnim, 'Position', [100, 100, 750, 600]);

hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Animación Control Cinemático (Saturación Proporcional)');

margin = 0.5;
xlim([min([q_con(1,:), xg]) - margin, max([q_con(1,:), xg]) + margin]);
ylim([min([q_con(2,:), yg]) - margin, max([q_con(2,:), yg]) + margin]);

plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot(xg, yg, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Objetivo');

hPath = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot');
hRobot = plot(nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'HandleVisibility', 'off');

legend('Location', 'bestoutside');
paso_frame = 3;

for k = 1:paso_frame:N
    set(hPath, 'XData', q_con(1, 1:k), 'YData', q_con(2, 1:k));
    set(hRobot, 'XData', q_con(1, k), 'YData', q_con(2, k));
    drawnow;
    
    frame = getframe(figAnim);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    if k == 1
        imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', dt * paso_frame);
    else
        imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', dt * paso_frame);
    end
end

disp('------------------------------------------------------');
disp('¡Animación guardada como control_cinematico.gif!');