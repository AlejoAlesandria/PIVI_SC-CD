clear all
Amplitude = 2;
Ts = 0.01;
Order = 12;
NumPeriods = 1;

% Step 1: Generate PRBS signal
sign = frest.PRBS('Amplitude', Amplitude, 'Ts', Ts, 'Order', Order, 'NumPeriods', NumPeriods);

% Step 2: Create a time series of the PRBS signal
ts = generateTimeseries(sign);

% Step 3: Extract the PRBS data as a matrix
prbs_matrix = ts.Data;
writematrix(prbs_matrix', 'prbs_sequence.csv')
t = 0:Ts:Ts*length(prbs_matrix)-Ts;

% Display the matrix
%disp(prbs_matrix);
plot(t,prbs_matrix);
xlim([0 30]);