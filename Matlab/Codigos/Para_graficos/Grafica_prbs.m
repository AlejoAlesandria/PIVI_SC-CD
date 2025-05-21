Ts = 0.01;
prbs_sequence = csvread('prbs_sequence.csv');
t = 0:1:length(prbs_sequence)-Ts;

plot(prbs_sequence)
xlabel('Muestras')
ylabel('Amplitud')
title('Secuencia PRBS - Ts = 0.01 s')
xlim([0 length(prbs_sequence)])
ylim([-1.2 1.2])