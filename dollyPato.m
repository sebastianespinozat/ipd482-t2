close all;clear;clc;

load('dolly.mat')
load('ang_dolly.mat')

[x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO');


X = [DOLLYANGBUENO' DOLLYBUENO];
lidarData  = [x', y'];
x = x';
y = y';

[m, p] = aproximacion(lidarData, 500);

y_pred = p+m*x; 

tol = 0.1;
error = abs(y_pred - lidarData(:,2));
idx = find(error <= tol); 
A = lidarData(idx,:);

[m1, p1] = aproximacion(A, 50);
y_pred1 = p1+m1*x; 

figure
plot(x,y, '.')
hold on
plot(x, y_pred, 'r')
plot(A(:,1),A(:,2), '.g')
plot(x, y_pred1, 'm') 
legend('raw data', 'app 1', 'clean data', 'app 2')











function [a,b] = aproximacion(datos, numIter)    % y = a*x + b
    distThresh = 0.05;
    minInliers = 10;

    % Inicializar las variables para almacenar el mejor modelo y el n�mero m�ximo de inliers
    bestModel = [];
    maxInliers = 0;

    % Iterar sobre el n�mero de iteraciones
    for i = 1:numIter
        % Seleccionar aleatoriamente dos puntos
        points = datos(randperm(size(datos, 1), 2), :);

        % Calcular la l�nea que pasa por estos puntos (y = mx + b)
        m = (points(2, 2) - points(1, 2)) / (points(2, 1) - points(1, 1));
        b = points(1, 2) - m * points(1, 1);

        % Calcular las distancias de todos los puntos a esta l�nea
        dists = abs(m * datos(:, 1) - datos(:, 2) + b) / sqrt(m^2 + 1);

        % Encontrar los inliers (puntos dentro del umbral de distancia)
        inliers = find(dists <= distThresh);

        % Si el n�mero de inliers es mayor que el m�nimo requerido y mayor que el m�ximo actual, actualizar el mejor modelo y el m�ximo de inliers
        if length(inliers) > minInliers && length(inliers) > maxInliers
            bestModel = [m, b];
            maxInliers = length(inliers);
        end
    end

    % Mostrar el mejor modelo y el n�mero m�ximo de inliers
    disp(['Best model: y = ', num2str(bestModel(1)), 'x + ', num2str(bestModel(2))]);
    disp(['Max inliers: ', num2str(maxInliers)]);

    a = bestModel(1);
    b = bestModel(2);
end


