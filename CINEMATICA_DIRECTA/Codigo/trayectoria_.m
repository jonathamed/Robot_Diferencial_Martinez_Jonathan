%% SIMULACIÓN DE TRAYECTORIA EN "J" MAYÚSCULA VERTICAL
clear; clc; close all;

%% 1. PARÁMETROS DEL ROBOT
r = 0.05;   % Radio de la rueda en metros
L = 0.15;   % Distancia entre ruedas en metros

%% 2. INTERVALOS PARA LA "J" MAYÚSCULA
% [t_inicio, t_fin, theta_dot_derecha, theta_dot_izquierda]
intervalos = [
    0.0,  0.8,   10.0,    10.0;    % 1. Techo horizontal superior
    0.8,  1.3,   -4.7124,  4.7124; % 2. Giro exacto de 90° hacia abajo
    1.3,  3.3,   10.0,    10.0;    % 3. Bajada vertical recta
    3.3,  4.87,   2.0,     8.0;    % 4. Gancho curvo inferior
    4.87, 5.5,    8.0,     8.0     % 5. Remate recto subiendo
    ];

%% 3. CONFIGURACIÓN INICIAL Y PASO RÁPIDO
dt = 0.01;            % Paso de integración (10 ms)
x = 0; y = 0; th = 0; % Pose inicial

hist_x = x;
hist_y = y;
hist_th = th;

%% 4. INTEGRACIÓN NUMÉRICA
for i = 1:size(intervalos, 1)
    t_i = intervalos(i, 1);
    t_f = intervalos(i, 2);
    w_R = intervalos(i, 3); 
    w_L = intervalos(i, 4); 

    u = (r / 2) * (w_R + w_L);
    w = (r / L) * (w_R - w_L);

    for t = t_i:dt:(t_f - dt)
        x = x + u * cos(th) * dt;
        y = y + u * sin(th) * dt;
        th = th + w * dt;

        hist_x(end+1) = x;
        hist_y(end+1) = y;
        hist_th(end+1) = th;
    end
end

%% 5. GENERAR Y GUARDAR FIGURA (.PNG)
fig = figure('Name', 'Trayectoria J Mayuscula', 'Color', 'w');
plot(hist_x, hist_y, 'b-', 'LineWidth', 2.5); hold on;
plot(hist_x(1), hist_y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Inicio
plot(hist_x(end), hist_y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Fin

grid on; axis equal;
title('Trayectoria Libre en Forma de "J" Mayúscula');
xlabel('Posición X (m)'); ylabel('Posición Y (m)');
legend('Trayectoria J', 'Inicio', 'Fin', 'Location', 'best');

% Exportar imagen PNG solicitada
saveas(fig, 'trayectoria_libre.png');
disp('✔ Imagen exportada correctamente: trayectoria_libre.png');

%% 6. GENERAR Y GUARDAR ANIMACIÓN RÁPIDA (.GIF)
gif_filename = 'trayectoria_libre.gif';
step_anim = 10; % Salto para renderizar súper rápido el GIF

for k = 1:step_anim:length(hist_x)
    clf(fig);
    plot(hist_x(1:k), hist_y(1:k), 'b--', 'LineWidth', 1.8); hold on;

    % Flecha de dirección del robot
    rx = hist_x(k); ry = hist_y(k); rth = hist_th(k);
    quiver(rx, ry, cos(rth)*0.08, sin(rth)*0.08, 0, 'MaxHeadSize', 0.8, 'Color', 'r', 'LineWidth', 2);

    grid on; axis equal;
    xlim([min(hist_x)-0.1, max(hist_x)+0.1]);
    ylim([min(hist_y)-0.1, max(hist_y)+0.1]);
    title(sprintf('Animación "J" - Tiempo: %.2f s', (k-1)*dt));
    xlabel('Posición X (m)'); ylabel('Posición Y (m)');
    drawnow;

    % Captura para el GIF
    frame = getframe(fig);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    if k == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
    end
end
disp('✔ Animación exportada correctamente: trayectoria_libre.gif');
