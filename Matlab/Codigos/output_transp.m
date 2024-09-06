data = readtable('output.csv');
% Convertir la tabla a una matriz
matriz = table2array(data);
prbs_multi = prbs_matrix*4095
% Transponer la matriz para que los datos queden en horizontal
% matriz_horizontal = matriz';

% Guardar los datos transpuestos en un nuevo archivo CSV
% writematrix(matriz_horizontal, 'output_trans.csv');

% angle = matriz(:,1);
% freq = matriz(:,2);
