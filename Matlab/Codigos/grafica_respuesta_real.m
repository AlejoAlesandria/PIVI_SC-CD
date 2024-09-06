%% 
clear all;
data = readtable('respuesta_impulso.csv');
data1 = readtable('respuesta_escalon.csv');

Ts = 0.01;

impulse_response = table2array(data)-153;
step_response = table2array(data1)-153;

% Crear vector de tiempo
num_samples = length(impulse_response);
time = (0:num_samples-1) * Ts;
num_samples = length(step_response);
time1 = (0:num_samples-1) * Ts;

figure(1)
plot(time, impulse_response)
xlabel('Tiempo (s)')
ylabel('Amplitud (grados)')
title('Respuesta al impulso - Sistema real')
xlim([0 20])
ylim([-5 5])
grid on;

figure(2)
plot(time1, step_response)
xlabel('Tiempo (s)')
ylabel('Amplitud (grados)')
title('Respuesta al escalon - Sistema real')
xlim([0 20])
ylim([-2 11])
grid on;