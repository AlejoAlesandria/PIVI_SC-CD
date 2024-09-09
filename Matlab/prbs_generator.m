clear all;
% Parámetros
Ts = 0.01; % Tiempo de muestreo en segundos
fs = 1 / Ts; % Frecuencia de muestreo en Hz
f_max = 10; % Frecuencia máxima en Hz
NumPeriods = 30; % Mantener el número de períodos
Nu = 1; % Número de canales de la señal

% Aumentar el número de muestras por período
N = round(5 / (f_max * Ts)); % Aumentar el número de muestras por período de PRBS\
N = 200;

% Generación de la secuencia PRBS
Band = [0 f_max / fs]; % Rango de frecuencias normalizado
Range = [-1 1]; % Rango de la señal

prbs_sequence = idinput([N, Nu, NumPeriods], 'prbs', Band, Range);

% Creación del vector de tiempo
time = (0:length(prbs_sequence')-1) * Ts;

% Guardar la secuencia PRBS en un archivo CSV
writematrix(prbs_sequence', 'prbs_sequence.csv');

% Graficar la secuencia PRBS
plot(time, prbs_sequence);
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Secuencia PRBS Extendida con Más Muestras por Período');
grid on;
