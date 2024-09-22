data = readtable('Identificacion1/output_positiva.csv');
prbs_sequence_positiva = load('Identificacion1/prbs_sequence_positiva2.csv')';
% Convertir la tabla a una matriz
matriz = table2array(data);
prbs_multi = prbs_sequence_positiva* 3.3;%(3.3-1.98)+1.98;
% Transponer la matriz para que los datos queden en horizontal
% matriz_horizontal = matriz';

% Guardar los datos transpuestos en un nuevo archivo CSV
% writematrix(matriz_horizontal, 'output_trans.csv');

% angle = matriz(:,1);
% freq = matriz(:,2);
