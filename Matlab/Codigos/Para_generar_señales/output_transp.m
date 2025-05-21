data = readtable('output.csv');
%prbs_sequence_positiva = load('Identificacion15/prbs_sequence_positiva2.csv')';
% Convertir la tabla a una matriz
matriz = table2array(data) - 150;

data_in = readtable('prbs_sequence_ambos_sentidos.csv');
input = table2array(data_in);
input = input';

%Reemplazar los valores
input(input == -1) = -1638;
input(input == 1) = 1638;
% Transponer la matriz para que los datos queden en horizontal
% matriz_horizontal = matriz';

% Guardar los datos transpuestos en un nuevo archivo CSV
% writematrix(matriz_horizontal, 'output_trans.csv');

% angle = matriz(:,1);
% freq = matriz(:,2);
