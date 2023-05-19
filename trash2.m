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
MAX_cylindro = deg2rad(80);     %tan(0.4/0.1);
auxCylinder = zeros(1,1);
auxAngCylinder = zeros(1,1);
justStarted = 1;

% Parametros Dolly
m_anterior = 0;
stateDolly = 'front';

% figure()
set(figure(),'WindowStyle','docked') % Insert the figure to dock
for i=4500:L
    sprintf("Iteracion: %d", i)
    
    %######## CILINDRO ##########%
    % Filtrado por radio de CILINDRO
    indicesCilindro = find(SCANCOPY{i}.Ranges >= 0.4123 & SCANCOPY{i}.Ranges <= 0.55);
    CYLINDER = SCANCOPY{i}.Ranges(indicesCilindro);
    CYLINDER_ANGLES = angulos(indicesCilindro)';
    
    % Filtrado por angulo de CILINDRO
    indicesAngCilindro = find(CYLINDER_ANGLES <= MAX_cylindro & CYLINDER_ANGLES >= -MAX_cylindro);
    CYLINDER = CYLINDER(indicesAngCilindro);
    CYLINDER_ANGLES = CYLINDER_ANGLES(indicesAngCilindro);
    
        %%% CYLINDER_ANGLES y CYLINDER se sobreescriben en cada iteraci�n para
        %%% calculos de ECM
    
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
    auxMeanRad = mean(auxCylinder);
    auxMeanAng = mean(auxAngCylinder);

    
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
    [m, p] = aproximacion(lidarDataCART, 25);
    y_pred = p+m*x; 
   
    disp(['Error pendienteDOLLY ("k" - "k-1"):  ', num2str(abs(m - m_anterior))]);
    
    % Verificacion Cara Lateral (error de pendiente)
%     if abs(m - m_anterior) > 0.8    &&  i~=1
%         [m_inf, p_inf] = aproximacion(lidarDataCART(1:floorDiv(lenLidar,2),:), 25);
%         [m_sup, p_sup] = aproximacion(lidarDataCART(floorDiv(lenLidar,2)+1 :end , :), 25);
%         
%         if abs(m_anterior - m_inf) < 0.8
%             m = m_inf;
%             p = p_inf;
%         else
%             m = m_sup;
%             p = p_sup;
%         end
%         
%         y_pred = p+m*x;
%     end
    
    % Verificacion Cara Lateral (error de pendiente) (Divide hasta 4 partes)
%     if abs(m - m_anterior) > 0.8    &&  i~=1
%         [m_inf, p_inf] = aproximacion(lidarDataCART(1:floorDiv(lenLidar,2),:), 25);
%         [m_sup, p_sup] = aproximacion(lidarDataCART(floorDiv(lenLidar,2)+1 :end , :), 25);
%         
%         if abs(m_anterior - m_inf) < 0.8
%             m = m_inf;
%             p = p_inf;
%         elseif abs(m_anterior - m_sup) < 0.8
%             m = m_sup;
%             p = p_sup;
%         else
%             [m_inf1, p_inf1] = aproximacion(    lidarDataCART(1:floorDiv(lenLidar,4),   :)                              , floorDiv(25,2));
%             [m_inf2, p_inf2] = aproximacion(    lidarDataCART(floorDiv(lenLidar,4)+1    :2*floorDiv(lenLidar,4),   :)   , floorDiv(25,2));
%             [m_sup1, p_sup1] = aproximacion(    lidarDataCART(2*floorDiv(lenLidar,4)+1  :3*floorDiv(lenLidar,4),   :)   , floorDiv(25,2));
%             [m_sup2, p_sup2] = aproximacion(    lidarDataCART(3*floorDiv(lenLidar,4)+1  :end ,                     :)   , floorDiv(25,2));
%         
%             if abs(m_anterior - m_inf1) < 0.8
%                 m = m_inf1;
%                 p = p_inf1;
%             elseif abs(m_anterior - m_inf2) < 0.8
%                 m = m_inf2;
%                 p = p_inf2;
%             elseif abs(m_anterior - m_sup1) < 0.8
%                 m = m_sup1;
%                 p = p_sup1;
%             elseif abs(m_anterior - m_sup2) < 1
%                 m = m_sup2;
%                 p = p_sup2;
%             end
%         end
%         y_pred = p+m*x;
%     end
    
    
    % Identificacion cara lateral (SUPUESTO: se inicia el programa viendo la cara frontal verdadera)
    if m_anterior*m > -1.5 && m_anterior*m < -0.5 && i ~= 1
        if contains(stateDolly, 'front')
           stateDolly = 'lat'; 
        else
            stateDolly = 'front';
        end
    end
    disp(['Cara: ', stateDolly])
    disp(['front*lat: ', num2str(m_anterior*m)])
    
    
    % Obtencion rectas Lidar y extension a cilindro
%     [x_cart1, y_cart1] = pol2cart(auxMeanAng,auxMeanRad);
%     m_aux1 = (0 - y_cart1) / (0 - x_cart1);
%     b_aux1 = y_cart1 - m_aux1 * x_cart1;
    radioLinea = 0:0.1:1.5;
    angLinea1 = ones(length(radioLinea))*max(CYLINDER_ANGLES);
    angLinea2 = ones(length(radioLinea))*min(CYLINDER_ANGLES);
    
    
    
    
    % Filtraje puntos interes con Recta
    tol = 0.1;
    error = abs(y_pred - lidarDataCART(:,2));
    idx = find(error <= tol); 
    dollyPOINTS = lidarDataCART(idx,:);
    
    % Guardado de pendiente correcta para iteracion siguiente
    m_anterior = m;
    
    % Puntos finales a coordenadas Polares
    [dollyANG, dollyRADIO] = cart2pol(dollyPOINTS(:,1), dollyPOINTS(:,2));

    
    %######### GRAFICAS ########%
    polarplot([0 0], [0 0.1], '-*')                 % Lidar
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')    % Puntos
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')      % Cilindro
    
    polarplot(angLinea1, radioLinea, 'r-')   % Linea Lidar al Cilindro
    polarplot(angLinea2, radioLinea, 'r-')   % Linea Lidar al Cilindro
    
    if contains(stateDolly, 'front')
        polarplot(dollyANG, dollyRADIO, 'g.')         % Dolly
    else
        polarplot(dollyANG, dollyRADIO, 'm.')         % Dolly
    end
%     polarplot(dollyANG, dollyRADIO, 'g.')           % Dolly
   
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)   % Rango min Cilindro
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)    % Rango max Cilindro
    rlim([0 1.5])                                   % Rango max Dolly
    pause(1e-5)
    hold off
    
end







%% Funciones

function [a,b] = aproximacion(datos,minInliers)    % y = a*x + b
    numIter = 500;
    distThresh = 0.01;  % dist max a la recta optima
%     minInliers = 25;    % Minimo Numero de puntos para recta optima
    error = 1e-2;

    % Inicializar las variables para almacenar el mejor modelo y el número máximo de inliers
    bestModel = [];
    maxInliers = 0;

    % Iterar sobre el número de iteraciones
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
        
        % Si el número de inliers es mayor que el mínimo requerido y mayor que el máximo actual, actualizar el mejor modelo y el máximo de inliers
        if length(inliers) > minInliers && length(inliers) > maxInliers            
            bestModel = [m, b];
            maxInliers = length(inliers);
        end
        
    end

    % Mostrar el mejor modelo y el número máximo de inliers
    disp(['Best model: y = ', num2str(bestModel(1)), 'x + ', num2str(bestModel(2))]);
    disp(['Max inliers: ', num2str(maxInliers)]);

    a = bestModel(1);   % m
    b = bestModel(2);   % b
end
