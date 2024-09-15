clear all;
close all;
%% Parámetros de Simulación
Ts = 0.01;
Tf = 30;
Nsim = Tf/Ts;
t = 0:Ts:Tf;

%% Señal de Entrada - Set Point
r_sp = 2*ones(size(t));

%% Espacio de estados identificado invertido - ecuacion calculada
% A = [0 1 0; 4.8976 -0.52198 0.319; 0 0 -0.33792];
% B = [0; -0.468; 0.495];
% C = [1 0 0];
% D = 0;

A = [0 1 0; 0 0 1; -0.4588 -12.83 -3.986];
B = [-0.00008368; -0.05376; 0.01643];
C = [1 0 0];
D = 0;

% A = [0 1; -0.9903 1.99];
% B = [-0.000002972; -0.000009109];
% C = [1 0];
% D = 0;

% Funcion de transferencia Pendulo natural
% num = [-0.001862 -0.05642];
% den = [1 0.4719 4.919];
% [A, B, C, D] = tf2ss(num,den);
% 
sys_ss = ss(A, B, C, D);

[SS_disc] = c2d(sys_ss, Ts);

% SS_disc = ss(A, B, C, D, Ts);
% sys_ss = d2c(SS_disc);
%% Tamaño vectores Espacio de Estados
nx = length(SS_disc.A);

%% Asignación de Polos
pole1 = -0.1+0.8j;
pole2 = -0.1-0.8*j;
pole3 = 0.9;
pole4 = -0.7;
p = [pole1 pole2 pole3 pole4];

%% Determinacion de la matriz K - caso en donde existe integrador
%%[K, prec] = place(SS_disc.A, SS_disc.B, p);

%% Determinación de la matriz K - caso en donde no existe integrador 
% Servosistema tipo 1 planta sin integrador
A_hat = [SS_disc.A zeros(nx, 1); -SS_disc.C 0];
B_hat = [SS_disc.B; 0];
p_new = p;
%K_hat = place(A_hat, B_hat, p_new);
K_hat = place(A_hat,B_hat, p);
K_new = K_hat(1:nx);
ki = K_hat(end);

%% Variables Iniciales
x0 = zeros(1, nx)';
x = ones(nx, Nsim + 1) .* x0;

% Observador
x_hat = ones(nx, Nsim + 1) .* x0;
L = [0.2;0.2;0.2]; % Ganancia del observador

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
    % Microcontrolador %
    e(k) = r_sp(k) - y_feedback;

    % Sin integrador  
%     u(k) = - K*x(:, k);

    % Con integrador, sin referencia
%     e(k) = y_feedback;
%     u(k) = -ki*y_feedback - K_new*x(:, k);

    % Con integrador
    u(k) = -ki*q(k) - K_new*x_hat(:, k);
   
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
for i = 1:nx
    plot(t, x_hat(i, 1:Nsim+1));
end
for i = 1:nx
    plot(t, x(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)','x(1)','x(2)');
title('Estimación de los estados x_hat');
xlabel('Tiempo [s]');
ylabel('Estados estimados');
grid on