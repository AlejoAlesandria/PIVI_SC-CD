% Parámetros
t_total = 80; % Tiempo total en segundos (10s en 1, 3s en 0, 10s en -1)
dt = 0.01;    % Paso de tiempo (frecuencia de muestreo)
t = 0:dt:t_total; % Vector de tiempo

% Generación de la señal escalón con transición a 0
step_signal = ones(size(t)); % Inicializa la señal en 1
step_signal(t > 10 & t <= 20) = 0;  % Poner en 0 entre los 10s y 13s
step_signal(t > 20 & t <= 30) = -1;           % Cambia a -1 después de los 13s
step_signal(t > 30 & t <= 40) = 0; % Inicializa la señal en 1
step_signal(t > 40 & t <= 50)= 1;  % Poner en 0 entre los 10s y 13s
step_signal(t > 50 & t <= 60)= 0;          % Cambia a -1 después de los 13s
step_signal(t > 60 & t <= 70)= -1;
step_signal(t > 70 & t <= 80)= 0;

% Guardar los valores en un archivo CSV
data = [step_signal];  % Concatenar tiempo y señal en columnas
csvwrite('step_signal_zero_transition.csv', data);  % Guardar en archivo CSV

% Gráfica de la señal
figure;
plot(t, step_signal, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Señal escalón con transición de 0 entre 1 y -1');
grid on;
