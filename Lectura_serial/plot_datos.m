% Cargar los datos desde el archivo CSV
filename = 'data_20240925_080805.csv'; % Nombre del archivo
data = readtable(filename);

% Extraer las columnas de interés
Time = data.Time;
U = data.U;
Y = data.Y;
X1 = data.X1;
X2 = data.X2;
X3 = data.X3;

% Crear las gráficas
figure(1);

% Gráfica de la señal U
subplot(2,1,1);
plot(Time, U);
title('Señal de entrada U');
xlabel('Tiempo (s)');
ylabel('U');
grid on;

% Gráfica de la salida Y
subplot(2,1,2);
plot(Time, Y);
title('Salida del sistema Y');
xlabel('Tiempo (s)');
ylabel('Y');
grid on;

% Gráfica de los estados X1, X2, X3
figure(2)
subplot(3,1,1);
plot(Time, X1);
title('Estado X1');
xlabel('Tiempo (s)');
ylabel('X1');
grid on;

% Gráfica de la salida Y
subplot(3,1,2);
plot(Time, X2);
title('Estado X2');
xlabel('Tiempo (s)');
ylabel('X2');
grid on;

subplot(3,1,3);
plot(Time, X3);
title('Estado X3');
xlabel('Tiempo (s)');
ylabel('X3');
grid on;

% plot(Time, X1, '-r', Time, X2, '-b');
% title('Estados del sistema X1, X2');
% xlabel('Tiempo (s)');
% ylabel('Estados');
% legend('X1', 'X2');
% % ylim([-50 50]);
% grid on;
