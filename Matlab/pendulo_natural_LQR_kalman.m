clear all;
close all;
%% Parámetros de Simulación
Ts = 0.01;
Tf = 15;
Nsim = Tf/Ts;
t = 0:Ts:Tf;

%% Señal de Entrada - Set Point
r_sp = zeros(size(t));

%% Espacio de estados identificado invertido - ecuacion calculada
% A = [0 1 0; 0 0 1; -0.4588 -12.83 -3.986];
% B = [-0.00008368; -0.05376; 0.01643];
% C = [1 0 0];
% D = 0;

A = [0 1 0; 0 0 1; -1112 -99.35 -295.8];
B = [0.1053; -35.72; 10520];
C = [1 0 0];
D = 0;

% A = [0 1; -3.513 -0.1999];
% B = [0.001527; 0.01147];
% C = [1 0];
% D = 0;

sys_ss = ss(A, B, C, D);

SS_disc = c2d(sys_ss, Ts);

% SS_disc = ss(A, B, C, D, Ts);
% sys_ss = d2c(SS_disc);
%% Tamaño vectores Espacio de Estados
nx = length(SS_disc.A);

%% Determinación de la matriz K - LQR
Q = diag([1000000000 1 1 100000]);
R = 0.0000000001;
% Q = diag([10000000 1000 10000]);
% R = 0.001;
% Q = diag([10000000 10000 1000]);
% R = 0.001;

% Q = diag([10000 100 1]);
% R = 1;

K_hat = lqi(SS_disc, Q, R);
K_new = K_hat(1:nx);
ki = K_hat(end);

%% Observador mediante Kalman
x0 = zeros(1, nx)';
x_hat = ones(nx, Nsim + 1) .* x0; % Estados estimados

% Definir las matrices de covarianza del proceso y de la medición
Q_kalman = 10 * eye(nx);  % Matriz de covarianza del ruido del proceso
R_kalman = 0.001;            % Covarianza del ruido de medición (ajústalo según el sistema)
P_kalman = eye(nx);               % Matriz de covarianza inicial
P_kalman_pred = eye(nx).*0;

%% Variables Iniciales
x = ones(nx, Nsim + 1) .* x0;

% Salidas del simulador
yOut  = SS_disc.C*x0; 
tOut = 0;
uOut = 0;
dt = linspace(0, Ts, 10)';
integral_error = zeros(1, Nsim);
e = zeros(1, Nsim);
q = zeros(1, Nsim);
y_feedback = SS_disc.C*x0;
%% Representa el tiempo real, cuanto tiempo está corriendo el
% microcontrolador, el tiempo entre interrupciones el tiempo de muestreo
for k = 1:Nsim
%     if k >= 5/Ts
%         r_sp(k) = 0;
%         x(:,k) = [21; -593.76; 145763.12];
%     end
%     if k >= 7/Ts
%         x(:,k) = [0;-153.71;25753.42];
%     end
    if k >= 5/Ts
        r_sp(k) = 3;
    end
    % 5.28,-4095.0,21.0,21.0,-593.76,145763.12
    % 6.94,137.0,0.0,-0.0,-153.71,25753.42
    % Microcontrolador %
    e(k) = r_sp(k) - y_feedback;

    % Planta sin integrador
    %u(k) = -ki*q(k) - K_new*x_hat(:, k);
    % Planta con integrador
       
    u(k) = -ki*q(k)- K_new*x_hat(:, k);
    if u(k) > 4095
        u(k) = 4095;
    elseif u(k) < -4095
        u(k) = -4095;
    end
    % Predicción del estado y de la covarianza
    x_hat(:, k) = SS_disc.A * x_hat(:, k) + SS_disc.B * u(k);
    P_kalman_pred = SS_disc.A * P_kalman * SS_disc.A' + Q_kalman;
    
    % Corrección (actualización) con la medición
    y_feedback = SS_disc.C * x(:, k);
    y_hat = SS_disc.C * x_hat(:, k);
    K_kalman = P_kalman_pred * SS_disc.C' / (SS_disc.C * P_kalman_pred * SS_disc.C' + R_kalman);
    x_hat(:, k+1) = x_hat(:, k) + K_kalman * (y_feedback - y_hat);
    P_kalman = (eye(nx) - K_kalman * SS_disc.C) * P_kalman_pred;
    
    uOut = [uOut u(k)];
    
    u = u(k)*ones(1, numel(dt)); % ZOH de la U - DAC

    % Sistema en la vida real %
    [y, tsim, XssOut] = lsim(sys_ss, u, dt, x(:, k)); % XssOut es la salida del Espacio de Estados. Cambiar
    x(:, k+1) = XssOut(end, :)'; % x es el vector de estados
    q(k+1) = q(k) + e(k); % Integrador se obtiene con el modelo discretizado
    y_feedback = y(end, :); % Sampling - ADC

    tOut = [tOut (tsim' + t(k))];
    yOut = [yOut y'];
end

%% Gráficos
figure(1)
subplot(2,1,1)
plot(tOut, yOut);
hold on
stairs(t, SS_disc.C*x);
plot(t, r_sp);
legend('output', 'sampled', 'set-point')
title('Salida del sistema y');
ylabel('y');
ylim([-0.5 4]);
grid on

subplot(2,1,2)
hold on
plot(t, uOut)
legend('u signal')
title('Señal de entrada u');
ylabel('u');
grid on

%% Gráficos de la estimación de los estados
figure(2)
hold on
subplot(2,1,1)
    hold on
    for i = 1:nx
        plot(t, x_hat(i, 1:Nsim+1));
    end
    legend('x_{hat}(1)','x_{hat}(2)', 'x_{hat}(3)');
    grid on
    title('Estimación de los estados x_{hat}');
    xlabel('Tiempo [s]');
    ylabel('Estados estimados');
subplot(2,1,2)
    hold on
    for i = 1:nx
        plot(t, x(i, 1:Nsim+1));
    end
    legend('x(1)','x(2)','x(3)');
    grid on
    title('Estados reales x');
    xlabel('Tiempo [s]');
    ylabel('Estados estimados');

%% Gráficos de la estimación de los estados junto
figure(3)
hold on
for i = 1:nx
    plot(t, x_hat(i, 1:Nsim+1));
end
for i = 1:nx
    plot(t, x(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)', 'x_{hat}(3)', 'x(1)','x(2)','x(3)');
grid on
title('Estados reales x y estimados x_{hat}');
xlabel('Tiempo [s]');
ylabel('Estados estimados');

figure(4)
hold on
subplot(3,1,1)
plot(t,x_hat(1,1:Nsim+1));
legend('x_{hat}(1)');
grid on
title('Estados estimado x_{hat}(1)');
xlabel('Tiempo [s]');
ylabel('x_{hat}(1)');
ylim([-0.5 4]);


subplot(3,1,2)
plot(t,x_hat(2,1:Nsim+1));
legend('x_{hat}(2)');
grid on
title('Estados estimado x_{hat}(2)');
xlabel('Tiempo [s]');
ylabel('x_{hat}(2)');

subplot(3,1,3)
plot(t,x_hat(3,1:Nsim+1));
legend('x_{hat}(3)');
grid on
title('Estados estimado x_{hat}(3)');
xlabel('Tiempo [s]');
ylabel('x_{hat}(3)');