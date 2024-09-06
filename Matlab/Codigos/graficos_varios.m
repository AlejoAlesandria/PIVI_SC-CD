clear all;
load('funcion_transferencia.mat');
H = tf(tf3.Numerator, tf3.Denominator);
% H = c2d(H, 0.01);
figure(1)
impulse(H);
xlabel('Tiempo');
ylabel('Amplitud');
title('Respuesta al impulso')

figure(2)
step(H);
xlabel('Tiempo');
ylabel('Amplitud');
title('Respuesta al escalón')

figure(3)
pzmap(H)
xlim([-1 0.3])

figure(4)
rlocus(H)

figure(5)
bode(H)

figure(6)
nyquist(H)