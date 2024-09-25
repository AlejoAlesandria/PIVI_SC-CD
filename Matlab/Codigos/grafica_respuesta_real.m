%% 
clear all;
data = readtable('..\..\Lectura_serial\respuesta_escalon.csv');
data1 = readtable('..\..\Lectura_serial\respuesta_impulso.csv');

Ts = 0.01;

step_response = table2array(data)-153;
impulse_response = table2array(data1)-152;

% Crear vector de tiempo
num_samples = length(impulse_response);
time = (0:num_samples-1) * Ts;
num_samples = length(step_response);
time1 = (0:num_samples-1) * Ts;

figure(1)
plot(time, -impulse_response)
xlabel('Tiempo (s)')
ylabel('Amplitud (grados)')
title('Respuesta al impulso - Sistema real')
xlim([0 20])
ylim([-10 10])
grid on;

figure(2)
plot(time1, -step_response)
xlabel('Tiempo (s)')
ylabel('Amplitud (grados)')
title('Respuesta al escalon - Sistema real')
xlim([0 29.77])
ylim([-80 20])
grid on;