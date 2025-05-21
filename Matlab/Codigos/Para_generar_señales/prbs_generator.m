clear all;
% Parámetros
Ts = 0.01; % Tiempo de muestreo en segundos
fs = 1 / Ts; % Frecuencia de muestreo en Hz
f_max = 6; % Frecuencia máxima en Hz
NumPeriods = 10; % Mantener el número de períodos
Nu = 1; % Número de canales de la señal

% Aumentar el número de muestras por período
N = round(5 / (f_max * Ts)); % Aumentar el número de muestras por período de PRBS\
N = 1000;

% Generación de la secuencia PRBS
Band = [0 f_max / fs]; % Rango de frecuencias normalizado
Range = [-1 1]; % Rango de la señal

prbs_sequence_positiva = idinput([N, Nu, NumPeriods], 'prbs', Band, Range);
%prbs_sequence_negativa = prbs_sequence_positiva * -1;
% Creación del vector de tiempo
time = (0:length(prbs_sequence_positiva')-1) * Ts;

% Guardar la secuencia PRBS en un archivo CSV
writematrix(prbs_sequence_positiva', 'prbs_sequence_ambos_sentidos.csv');
%writematrix(prbs_sequence_negativa', 'prbs_sequence_negativa2.csv');

% Graficar la secuencia PRBS
plot(time, prbs_sequence_positiva);
hold on;
% plot(time, prbs_sequence_negativa);
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Secuencia PRBS Extendida con Más Muestras por Período');
grid on;
