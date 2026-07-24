clear; clc; close all;

% 5. Parametros del robot
r = 0.06;    % radio de rueda [m]
L = 0.37;    % distancia entre ruedas [m]
T = 10;      % tiempo total [s]
dt = 0.02;   % paso [s]
t = 0:dt:T;
x0 = 0; y0 = 0; theta0 = 0;

nombres = {
    'Caso 1', 'Caso 2', 'Caso 3', 'Caso 4', ...
    'Caso 5', 'Caso 6', 'Caso 7', 'Caso 8'
};

q_todas = cell(1, 8);

disp('================================================================================================');
disp('                                RESULTADOS                                   ');
disp('================================================================================================');
disp('Caso |  wD  |  wI  |  u [m/s] |  w [rad/s] |   x(end)   |   y(end)   | theta(end) ');
disp('------------------------------------------------------------------------------------------------');

for c = 1:8
    % Asignacion segun los datos EXACTOS de tu tabla de Word
    switch c
        case 1, wD = 10;  wI = 10;
        case 2, wD = -10; wI = -10;
        case 3, wD = 12;  wI = 6;
        case 4, wD = 6;   wI = 12;
        case 5, wD = 10;  wI = -10;
        case 6, wD = 10;  wI = 0;
        case 7, wD = 10;  wI = 8;
        case 8, wD = 10;  wI = 2;
    end
    
    % Calculo de u y w
    u = (r/2) * (wD + wI);
    w = (r/L) * (wD - wI);
    
    N = length(t);
    q = zeros(3, N);
    q(:,1) = [x0; y0; theta0];
    
    for k = 1:N-1
        dx = u * cos(q(3,k));
        dy = u * sin(q(3,k));
        dtheta = w;
        
        q(1, k+1) = q(1,k) + dx * dt;
        q(2, k+1) = q(2,k) + dy * dt;
        q(3, k+1) = q(3,k) + dtheta * dt;
    end
    
    q_todas{c} = q;
    
    % Imprimir exactamente la fila de tu tabla
    fprintf('%-4d | %-4d | %-4d | %-8.4f | %-10.4f | %-10.4f | %-10.4f | %-10.4f\n', ...
        c, wD, wI, u, w, q(1,end), q(2,end), q(3,end));
    
    % Grafica individual
    figure(c);
    plot(q(1,:), q(2,:), 'b-', 'LineWidth', 2); hold on;
    plot(q(1,1), q(2,1), 'go', 'MarkerFaceColor', 'g');
    plot(q(1,end), q(2,end), 'ro', 'MarkerFaceColor', 'r');
    grid on; axis equal;
    title(['Trayectoria Caso ' num2str(c)]);
    xlabel('X [m]'); ylabel('Y [m]');
    legend('Camino', 'Inicio', 'Fin');
end

% Grafica estatica de todas juntas
figure(9);
colores = lines(8);
for c = 1:8
    plot(q_todas{c}(1,:), q_todas{c}(2,:), 'LineWidth', 2, 'Color', colores(c,:));
    hold on;
end
grid on; axis equal;
title('Todas las Trayectorias de Cinemática Directa');
xlabel('X [m]'); ylabel('Y [m]');
legend(nombres, 'Location', 'bestoutside');

%% ========================================================================
%  GENERACIÓN DE ANIMACIÓN GIF (cinematica_directa.gif)
%  ========================================================================
filename = 'cinematica_directa.gif';
figAnim = figure(10);
set(figAnim, 'Position', [100, 100, 800, 600]); % Tamaño de ventana

% Crear arreglos de lineas para actualizar dinamicamente
linesAnim = zeros(1, 8);
hold on;
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Animación de Cinemática Directa');

for c = 1:8
    linesAnim(c) = plot(nan, nan, 'LineWidth', 2, 'Color', colores(c,:), 'DisplayName', nombres{c});
end
legend(nombres, 'Location', 'bestoutside');

% Determinar limites de los ejes
all_x = cellfun(@(v) v(1,:), q_todas, 'UniformOutput', false);
all_y = cellfun(@(v) v(2,:), q_todas, 'UniformOutput', false);
min_x = min(cellfun(@min, all_x)) - 0.2;
max_x = max(cellfun(@max, all_x)) + 0.2;
min_y = min(cellfun(@min, all_y)) - 0.2;
max_y = max(cellfun(@max, all_y)) + 0.2;
xlim([min_x, max_x]);
ylim([min_y, max_y]);

% Salto de cuadros para acelerar el renderizado
paso_frame = 5; 

for k = 1:paso_frame:N
    for c = 1:8
        set(linesAnim(c), 'XData', q_todas{c}(1, 1:k), 'YData', q_todas{c}(2, 1:k));
    end
    drawnow;
    
    % Capturar pantalla como imagen
    frame = getframe(figAnim);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Escribir archivo GIF
    if k == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', dt * paso_frame);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt * paso_frame);
    end
end

disp('¡Animación guardada exitosamente como cinematica_directa.gif!');
