load("SS_o3.mat");

[NUM, DEN] = ss2tf(ss2.A, ss2.B, ss2.C, ss2.D);

H_d = tf(NUM, DEN, 0.01);

H_c = d2c(H_d)

NUM = [-0.1128 0];
DEN = [1 0.3731 77.44 -0.02801];

HH = tf(NUM,DEN)