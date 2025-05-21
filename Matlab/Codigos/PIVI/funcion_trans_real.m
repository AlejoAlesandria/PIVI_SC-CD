NUM = [-0.1122 0];
DEN = [1 1.468 -74.2976 -61.8];

H = tf(NUM, DEN)

figure(1)
step(H)
xlabel('Tiempo');
ylabel('Amplitud');
title('Respuesta al escalon')
xlim([0 1]);

figure(2)
impulse(H)
xlabel('Tiempo');
ylabel('Amplitud');
title('Respuesta al impulso')
xlim([0 1]);

figure(3)
pzmap(H)

figure(4)
bode(H)

figure(5)
nyquist(H)