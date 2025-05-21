% Leer datos de output.csv
data = readtable('output22.csv');
% data2 = readtable('pid_pendulo_invertido.csv');

angle = table2array(data);
% angle2 = table2array(data2)-333;
Ts = 0.01;
num_samples = length(angle);
time = (0:num_samples-1) * Ts;

% num_samples2 = length(angle2);
% time2 = (0:num_samples2-1)*Ts;

figure(1)
plot(time, angle)
xlim([0 10]);
ylim([-4095 4095]);
%ylim([326 334])
xlabel('Tiempo (s)');
ylabel('Angulo (Grados)');
title('Respuesta de la planta con PID integrado');
grid on

% figure(1)
% plot(time2, angle2)
% xlim([0 42]);
% xlabel('Tiempo (s)');
% ylabel('Angulo (Grados)');
% title('Respuesta de la planta con PID integrado');
% grid on