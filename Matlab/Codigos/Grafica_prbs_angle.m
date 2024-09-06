clear all;
Ts = 0.01;
prbs_sequence = csvread('prbs_sequence.csv')*4095;

% Crear vector de tiempo
num_samples = length(prbs_sequence);
time = (0:num_samples-1) * Ts;

% Leer datos de output.csv
data = readtable('output.csv');
angle = table2array(data);

% Crear el gráfico
figure
hold on

% Subplot para prbs_sequence
subplot(2, 1, 1)
plot(time, prbs_sequence)
xlabel('Tiempo (s)')
ylabel('Amplitud (bit)')
title('Secuencia PRBS')
xlim([0 40.95])
ylim([-4500 4500])

% Subplot para angle
subplot(2, 1, 2)
plot(time, angle)
xlabel('Tiempo (s)')
ylabel('Ángulo (grados)')
title('Señal del encoder')
xlim([0 40.95])


