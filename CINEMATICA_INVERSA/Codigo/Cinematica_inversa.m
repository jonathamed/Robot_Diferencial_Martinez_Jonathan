clear; clc; close all;

% Parametros asignados
r = 0.06;    
L = 0.37;    
T = 10;      
dt = 0.02;   
t = 0:dt:T;
x0 = 0; y0 = 0; theta0 = 0;

% Parametros de la trayectoria circular
R = 1.2;      
wc = 0.5;     

% Cinemática inversa: cálculo de velocidades
u = R * wc;
w = wc;
wD = (u + (w * L / 2)) / r;
wI = (u - (w * L / 2)) / r;

% Simulación
N = length(t);
q = zeros(3, N);
q(:,1) = [x0; y0; theta0];

for k = 1:N-1
    q(1, k+1) = q(1,k) + u * cos(q(3,k)) * dt;
    q(2, k+1) = q(2,k) + u * sin(q(3,k)) * dt;
    q(3, k+1) = q(3,k) + w * dt;
end

% Impresión de resultados en consola
disp('=== RESULTADOS CINEMATICA INVERSA ===');
fprintf('Velocidad lineal u: %.4f m/s\n', u);
fprintf('Velocidad angular w: %.4f rad/s\n', w);
fprintf('Velocidad rueda derecha (wD): %.4f rad/s\n', wD);
fprintf('Velocidad rueda izquierda (wI): %.4f rad/s\n', wI);
fprintf('X final: %.4f m | Y final: %.4f m | Theta final: %.4f rad\n', ...
    q(1,end), q(2,end), q(3,end));

% Gráfica de la trayectoria circular (estática)
figStatic = figure(1);
plot(q(1,:), q(2,:), 'b-', 'LineWidth', 2); hold on;
plot(q(1,1), q(2,1), 'go', 'MarkerFaceColor', 'g');
plot(q(1,end), q(2,end), 'ro', 'MarkerFaceColor', 'r');
grid on; axis equal;
title('Cinemática Inversa - Trayectoria Circular');
xlabel('X [m]'); ylabel('Y [m]');
legend('Obtenida', 'Inicio', 'Fin');

% Exportar gráfica estática
saveas(figStatic, 'trayectoria_obtenida.png');

%% ========================================================================
%  GENERACIÓN DE ANIMACIÓN GIF (cinematica_inversa.gif)
%  ========================================================================
gifFilename = 'cinematica_inversa.gif';
figAnim = figure(2);
set(figAnim, 'Position', [100, 100, 700, 600]);

% Configuración del lienzo para la animación
hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Animación Cinemática Inversa - Trayectoria Circular');

% Límites fijos para evitar que la vista cambie durante la animación
margin = 0.3;
xlim([min(q(1,:)) - margin, max(q(1,:)) + margin]);
ylim([min(q(2,:)) - margin, max(q(2,:)) + margin]);

% Punto de inicio
plot(q(1,1), q(2,1), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');

% Elementos dinámicos (línea de rastro y posición actual)
hPath = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Trayectoria');
hRobot = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Robot');

legend('Location', 'bestoutside');

% Salto de fotogramas para optimizar tamaño y rendimiento
paso_frame = 4;

for k = 1:paso_frame:N
    % Actualizar trayectoria dibujada hasta el punto k
    set(hPath, 'XData', q(1, 1:k), 'YData', q(2, 1:k));
    % Actualizar posición actual del robot
    set(hRobot, 'XData', q(1, k), 'YData', q(2, k));
    
    drawnow;
    
    % Capturar pantalla
    frame = getframe(figAnim);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Guardar en el GIF
    if k == 1
        imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', dt * paso_frame);
    else
        imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', dt * paso_frame);
    end
end


disp('¡Animación guardada exitosamente como cinematica_inversa.gif!');
