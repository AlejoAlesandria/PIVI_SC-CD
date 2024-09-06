load("funcion_transferencia.mat");

[NUM, DEN] = ss2tf(ss2.A, ss2.B, ss2.C, ss2.D);

H = tf(NUM, DEN)