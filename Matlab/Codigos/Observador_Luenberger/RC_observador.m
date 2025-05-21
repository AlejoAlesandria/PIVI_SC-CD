clear all;
close all;
%% Parámetros de Simulación
Ts = 0.005;
Tf = 30;
Nsim = Tf/Ts;
t = 0:Ts:Tf;

%% Señal de Entrada - Set Point
r_sp = 2*ones(size(t));

%% Espacio de estados identificado invertido - ecuacion calculada
%% Modelo en Espacio de Estados MagLev
R = 10e3;
C = 10e-6;

A = [(-1/(R*C)-1/(R*C)) (1/(R*C));(1/(R*C)) (-1/(R*C))];
B = [1/(R*C);0];
C = [0 1];
D = [0];

sys_ss = ss(A, B, C, D);

[SS_disc] = c2d(sys_ss, Ts);

%% Tamaño vectores Espacio de Estados
nx = length(SS_disc.A);

%% Determinación de la matriz K - LQR
% Q = diag([100000 100 100]);
% R = 0.001;
% 
% K_hat = lqi(SS_disc, Q, R);
% K_new = K_hat(1:nx);
% ki = K_hat(end);

%% Asignación de Polos
pole1 = -0.1;
pole2 = -0.11;
pole3 = -0.111;

p = [pole1 pole2 pole3];

%% Determinacion de la matriz K - caso en donde existe integrador
% [K_new, prec] = place(SS_disc.A, SS_disc.B, p);
% ki = 0;
%% Determinación de la matriz K - caso en donde no existe integrador 
% Servosistema tipo 1 planta sin integrador
A_hat = [SS_disc.A zeros(nx, 1); -SS_disc.C 0];
B_hat = [SS_disc.B; 0];
p_new = p;
%K_hat = place(A_hat, B_hat, p_new);
K_hat = place(A_hat,B_hat, p);
K_new = K_hat(1:nx);
ki = K_hat(end);

%% Observador
x0 = zeros(1, nx)';

x_hat = ones(nx, Nsim + 1) .* x0;

% Polos en plano S
pole1_obs = -1;
pole2_obs = -10;
pole3_obs = -5;
% Polos mapeados a plano Z
pole1_obs_z = exp(pole1_obs*Ts);
pole2_obs_z = exp(pole2_obs*Ts);
pole3_obs_z = exp(pole3_obs*Ts);

p_obs = [pole1_obs_z pole2_obs_z];
L = place(SS_disc.A', SS_disc.C', p_obs);
L = L';

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
    
    if k >= 3/Ts
        r_sp(k) = 0;
    end
    if k >= 10/Ts
        r_sp(k) = 10;
    end
    % Microcontrolador %
    e(k) = r_sp(k) - y_feedback;

    % Sin integrador  
%     u(k) = - K*x(:, k);

    % Con integrador, sin referencia
%     e(k) = y_feedback;
%     u(k) = -ki*y_feedback - K_new*x(:, k);

    % Con integrador
    u(k) = -ki*q(k) - K_new*x_hat(:, k);
    %u(k) =  - K_new*x_hat(:, k);
    if u(k) > 3.3
        u(k) = 3.3;
    elseif u(k) < 0
        u(k) = 0;
    end
    x_hat(:, k+1) = (SS_disc.A - L*SS_disc.C)*x_hat(:, k) + SS_disc.B*u(k) + L*y_feedback;
     
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
plot(tOut, yOut);
hold on
stairs(t, SS_disc.C*x);
plot(t, r_sp);
legend('output', 'sampled', 'set-point')
grid on

figure(2)
hold on
plot(t, uOut)
legend('u signal')
grid on

%% Gráficos de la estimación de los estados
figure(3)
hold on
subplot(2,1,1)
    hold on
    for i = 1:nx
        plot(t, x_hat(i, 1:Nsim+1));
    end
    legend('x_{hat}(1)','x_{hat}(2)','x_{hat}(3)');
    grid on
    title('Estimación de los estados x_hat');
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
figure(4)
hold on
for i = 1:nx
    plot(t, x_hat(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)','x_{hat}(3)');
for i = 1:nx
    plot(t, x(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)','x_{hat}(3)','x(1)','x(2)','x(3)');
grid on
title('Estados reales x y estimados x_{hat}');
xlabel('Tiempo [s]');
ylabel('Estados estimados');