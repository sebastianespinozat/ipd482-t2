clc;clear;close all;

%####### ANALISIS TIEMPO REAL LIDAR #########%
%####### DETECCION CILINDRO ######%
%####### DETECCION DOLLY POR CANTIDAD DE PUNTOS EN RECTA ######%

% Carga de datos LIDAR
bag = rosbag('laser_data.bag');
scan = bag.readMessages('DataFormat', 'struct');
SCANCOPY = scan;

% Parametros de interes (radio max y angulo minimo de deteccion)
L = length(scan);
rLIM = 1.5;     
angulos = linspace(scan{1}.AngleMin, scan{1}.AngleMax, length(scan{1}.Ranges));

% Parametros CILINDRO
MAX_cylindro = deg2rad(80);   
auxCylinder = zeros(1,1);
auxAngCylinder = zeros(1,1);
justStarted = 1;

% Parametros iniciales Dolly
m_anterior = 0;
stateDolly = 'front';   % 'front' o 'lat'
    
% Ang y Coord Buscadas
B1 = zeros(L, 1);
x1 = zeros(L, 1);
y1 = zeros(L, 1);
B2 = zeros(L, 1);
x2 = zeros(L, 1);
y2 = zeros(L, 1);

tiempo = zeros(L,1) ; 

inicio = 1;
staph = 7400;
%  figure()
set(figure(),'WindowStyle','docked') % Insert the figure to dock
for i=inicio:staph
    
    sprintf("Iteracion: %d", i)
    tic
    %######## CILINDRO ##########%
    % Filtrado por radio de CILINDRO
    indicesCilindro = find(SCANCOPY{i}.Ranges >= 0.4123 & SCANCOPY{i}.Ranges <= 0.55);
    CYLINDER = SCANCOPY{i}.Ranges(indicesCilindro);
    CYLINDER_ANGLES = angulos(indicesCilindro)';
    
    % Filtrado por angulo de CILINDRO
    indicesAngCilindro = find(CYLINDER_ANGLES <= MAX_cylindro & CYLINDER_ANGLES >= -MAX_cylindro);
    CYLINDER = CYLINDER(indicesAngCilindro);
    CYLINDER_ANGLES = CYLINDER_ANGLES(indicesAngCilindro);
    
        %%% CYLINDER_ANGLES y CYLINDER se sobreescriben en cada iteracion 
        %%% para calculos de ECM
    
    % Copia de puntos de interes
    CYLINDER_ANGLES_i = CYLINDER_ANGLES;
    CYLINDER_i = CYLINDER;
    
%     sprintf("CANT PUNTOS: %d", length(CYLINDER_ANGLES))
%     sprintf("CANT PUNTOS AUX: %d", length(auxAngCylinder))
    
    % Parametros para ECM de Cilindro
    maxAng = max(CYLINDER_ANGLES_i);
    minAng = min(CYLINDER_ANGLES_i);
    lowA = minAng;
    supA = maxAng;
    ECM = immse(CYLINDER_ANGLES_i, mean(CYLINDER_ANGLES_i).*ones(1,length(CYLINDER_ANGLES_i))');
    
    idx = 1:1:length(CYLINDER_ANGLES);
    idxLow = zeros(1,100);
    idxSup = zeros(1,100);
    
    % Si NO capta puntos de interes (se mantiene cilindro anterior)
    if (isempty(CYLINDER_ANGLES))
     
        CYLINDER = auxCylinder;
        CYLINDER_ANGLES = auxAngCylinder;
    
    else
        % Filtrado Cilindro por ECM (divide el angulo en 2 hasta una cota)
        while (ECM > 1e-3)%

            splitAng = mean([lowA supA]);
            diffAng = abs(lowA - supA);

            idxLow = find(CYLINDER_ANGLES_i < splitAng);
            idxSup = find(CYLINDER_ANGLES_i >= splitAng);

            ecmL = immse(CYLINDER_ANGLES_i(idxLow), mean(CYLINDER_ANGLES_i(idxLow)*ones(1,length(CYLINDER_ANGLES_i(idxLow))))');
            ecmS = immse(CYLINDER_ANGLES_i(idxSup), mean(CYLINDER_ANGLES_i(idxSup)*ones(1,length(CYLINDER_ANGLES_i(idxSup))))');    
            if ecmL < ecmS 
                ECM = ecmL;
                lowA = splitAng - diffAng/2;
                supA = splitAng;
                idx = idxLow;
            else
                ECM = ecmS;
                lowA = splitAng;
                supA = splitAng + diffAng/2;
                idx = idxSup;
            end

        end
        % Cilindro detectado
        CYLINDER_ANGLES = CYLINDER_ANGLES_i(idx);
        CYLINDER = CYLINDER_i(idx);
        
        % Para iteracion 1
        if (justStarted == 1)
            auxMeanAng = mean(CYLINDER_ANGLES);
            justStarted = 0;
        end
        % Si esta muy alejado del cilindro anterior, se descarta (sobre 10° || un angulo de cilindro inferior)
        if (abs(mean(CYLINDER_ANGLES) - auxMeanAng) > deg2rad(10) || length(CYLINDER_ANGLES) < 6)
            CYLINDER = auxCylinder;
            CYLINDER_ANGLES = auxAngCylinder;
        end
    end

    % Guardado de cilindro para iteracion siguiente
    auxCylinder = CYLINDER;
    auxAngCylinder = CYLINDER_ANGLES;
    auxMeanRatio = mean(auxCylinder);
    auxMeanAng = mean(auxAngCylinder);
     
    % Calculo angulo Beta1
%     mA = auxAngCylinder(auxCylinder == min(auxCylinder));
%     [xR, yR] = pol2cart(mA, min(auxCylinder)); 
    [xR, yR] = pol2cart(auxMeanAng, min(auxCylinder)); 
    v_1 = [1,0,0] - [0,0,0];
    v_2 = [xR(1),yR(1),0] - [0.1,0,0];
    if yR(1) > 0
        ANGULO_B1 = atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    else
        ANGULO_B1 = -atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    end
    
%     ANGULO_B1 = atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    B1(i) = (ANGULO_B1);
%     B1(i,2) = rad2deg(asin(yR(1)/0.4));  VER COMO ARREGLAR ESTO +- 10�
%     DIFF
    % Calculo coordenada x1,y1
%     R1 = 1.097-0.4-0.05/2;
%     x1(i) = xR + (0.647+0.05)*cos(B1(i,1));
%     y1(i) = yR + (0.647+0.05)*sin(B1(i,1));
    x1(i) = 0.1+ 1.097*cos(B1(i,1));
    y1(i) = 1.097*sin(B1(i,1));


    %######## DOLLY ##########%
    % Filtrado por radio interes de DOLLY
    indicesDolly = find(SCANCOPY{i}.Ranges > 0.55 & SCANCOPY{i}.Ranges <= 1.5);
    DOLLY = SCANCOPY{i}.Ranges(indicesDolly);
    DOLLY_ANGLES = angulos(indicesDolly);

    % Filtrado por angulo de interes de DOLLY
    dollyANG_TOL = pi/6;
    indicesDolly = find(DOLLY_ANGLES < mean(CYLINDER_ANGLES) + dollyANG_TOL & DOLLY_ANGLES > mean(CYLINDER_ANGLES) - dollyANG_TOL);
    DOLLYBUENO = DOLLY(indicesDolly);
    DOLLYANGBUENO = DOLLY_ANGLES(indicesDolly)';
    
    % Coord Cartesianas Puntos Candidatos
    [x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO);
    lidarDataCART  = [x, y];
    lenLidar = length(lidarDataCART);
    
    % Regresion Lineal de todos los Datos
    [m, p] = aproximacion(lidarDataCART, 100);
    y_pred = p+m*x; 
    
    
    
    % Filtraje puntos interes con Recta
    tol = 0.01;
    error = abs(y_pred - lidarDataCART(:,2));
    idx = find(error <= tol); 
    dollyPOINTS = lidarDataCART(idx,:);

  
      
    % Puntos finales a coordenadas Polares
    [dollyANG, dollyRADIO] = cart2pol(dollyPOINTS(:,1), dollyPOINTS(:,2));
    m = RegLineal( dollyANG , dollyRADIO );

    % Identificacion cara lateral (SUPUESTO: se inicia el programa viendo la cara frontal verdadera)
    if m_anterior*m > -2.5 && m_anterior*m < -0.1 && i ~= 1
        if contains(stateDolly, 'front') 
           stateDolly = 'lat'; 
        else
            stateDolly = 'front';
        end
    end

     % Guardado de pendiente correcta para iteracion siguiente
    m_anterior = m;

    % Calculo Beta2
    if contains(stateDolly, 'front')    % Cuando capta cara frontal
        B2(i,1) = atan(-1/m);
    elseif contains(stateDolly, 'lat')  % Cuando capta cara lateral
        B2(i,1) = atan(m);
    end
    
    if i ~= 1 && (B2(i,1) - B2(i-1,1) > pi/2)   % Corrige cambios mayores a 90�
        B2(i,1) = -pi + B2(i,1);
        disp('CASO CUADRANTE 4')
    elseif i ~= 1 && (B2(i,1) - B2(i-1,1) < -pi/2)
        B2(i,1) = pi + B2(i,1);
        disp('CASO CUADRANTE 1')
    end
    
    
    % Calculo Coordenada x2,y2
    x2(i,1) = x1(i) + 0.78*cos(B2(i,1));
    y2(i,1) = y1(i) + 0.78*sin(B2(i,1));
    
    tiempo(i) = toc ; 
    %######### GRAFICAS ########%
    polarplot([0 0], [0 0.1], '-*')                 % Lidar
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')    % Puntos
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')      % Cilindro
    
    polarplot([0 auxMeanAng],[0.1 auxMeanRatio], '-')   
    
    if contains(stateDolly, 'front')
        polarplot(dollyANG, dollyRADIO, 'g.')         % Dolly
    else
        polarplot(dollyANG, dollyRADIO, 'm.')         % Dolly
    end
   
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)   % Rango min Cilindro
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)    % Rango max Cilindro
    rlim([0 1.5])                                   % Rango max Dolly
    pause(1e-5)
    hold off
    
end



%% Estimacion x1 y1 theta1
X = load('local_trailer_x.mat');
X = cell2mat(struct2cell(X));
Y = load('local_trailer_y.mat');
Y = cell2mat(struct2cell(Y));
theta = load('local_trailer_theta.mat');
theta = cell2mat(struct2cell(theta));


figure
subplot 321         % Theta 1
plot(B1(:,1), '.')  
hold on
plot(theta(:,1), '.')
legend('\theta_1','ref')
title('Theta 1 (\theta_1)')
xlim([inicio staph])
subplot 322         % Error Theta 1
plot(abs(B1(:,1)-theta(:,1)),'.')
xlim([inicio staph])
title("error angulo \theta_1")
subplot 323         % Coord x1
plot(x1, '.')
hold on
plot(X(:,1),'.')
legend('x1','ref')
xlim([inicio staph])
title('x1')
subplot 324         % Error Coord x1
plot(abs(X(:,1)-x1),'.')
xlim([inicio staph])
title("error X1")
subplot 325         % Coord y1
plot(y1, '.')
hold on; 
plot(Y(:,1),'.')
legend('y1','ref')
xlim([inicio staph])
title('y1')
subplot 326         % Error Coord y1
plot(abs(Y(:,1)-y1),'.')
xlim([inicio staph])
title("error Y1")


figure
subplot 321         % Theta 2
plot(B2(:,1), '.')  
hold on
plot(theta(:,2), '.')
legend('\theta_2','ref')
title('Theta 2 (\theta_2)')
xlim([inicio staph])
subplot 322         % Error Theta 2
plot(abs(B2(:,1)-theta(:,2)),'.')
xlim([inicio staph])
title("error angulo \theta_2")
subplot 323         % Coord x2
plot(x2, '.')
hold on
plot(X(:,2),'.')
legend('x2','ref')
xlim([inicio staph])
title('x2')
subplot 324         % Error Coord x2
plot(abs(X(:,2)-x2),'.')
xlim([inicio staph])
title("error X2")
subplot 325         % Coord y2
plot(y2, '.')
hold on; 
plot(Y(:,2),'.')
legend('y2','ref')
xlim([inicio staph])
title('y2')
subplot 326         % Error Coord y2
plot(abs(Y(:,2)-y2),'.')
xlim([inicio staph])
title("error Y2")



%% Funciones

function [a,b] = aproximacion(datos,minInliers)    % y = a*x + b
    numIter = 500;
    distThresh = 0.01;  % dist max a la recta optima
%     minInliers = 25;    % Minimo Numero de puntos para recta optima
    error = 1e-2;

    % Inicializar las variables para almacenar el mejor modelo y el numero maximo de inliers
    bestModel = [];
    maxInliers = 0;
    worstModel = [];
    worstMax = 0;
    % Iterar sobre el numero de iteraciones
    for i = 1:numIter
        % Seleccionar aleatoriamente dos puntos
        points = datos(randperm(size(datos, 1), 2), :);

        % Calcular la línea que pasa por estos puntos (y = mx + b)
        m = (points(2, 2) - points(1, 2)) / (points(2, 1) - points(1, 1));
        b = points(1, 2) - m * points(1, 1);

        % Calcular las distancias de todos los puntos a esta línea
        dists = abs(m * datos(:, 1) - datos(:, 2) + b) / sqrt(m^2 + 1);

        % Encontrar los inliers (puntos dentro del umbral de distancia)
        inliers = find(dists <= distThresh);

        % Break (error con bestModel anterior)
        if ~isempty(bestModel) && sum( abs(bestModel-[m, b])./abs(bestModel)) < error
            break
        end
        
        if length(inliers) > worstMax            
            worstModel = [m,b];
            worstMax = length(inliers);
        end
        
        % Si el número de inliers es mayor que el mínimo requerido y mayor que el máximo actual, actualizar el mejor modelo y el máximo de inliers
        if length(inliers) > minInliers && length(inliers) > maxInliers            
            bestModel = [m, b];
            maxInliers = length(inliers);
        end
        
 
        if i == numIter
            if maxInliers == 0
                bestModel = worstModel;
            end
        end
    end

    % Mostrar el mejor modelo y el numero maximo de inliers
%     disp(['Best model: y = ', num2str(bestModel(1)), 'x + ', num2str(bestModel(2))]);
%     disp(['Max inliers: ', num2str(maxInliers)]);

    a = bestModel(1);   % m
    b = bestModel(2);   % b
end

function m = RegLineal(angSupDolly, radSupDolly) 
    [xSup, ySup] = pol2cart(angSupDolly, radSupDolly);
        m = polyfit(xSup , ySup, 1);
        m = m(1) ; 
end
