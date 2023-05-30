clc;clear;close all;

%####### ANALISIS TIEMPO REAL LIDAR #########%
%####### DETECCION CILINDRO ######%
%####### DETECCION DOLLY POR CLASIFICACION DE GRUPOS ######%

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

% Ang y Coord Buscadas
B1 = zeros(L, 1);
x1 = zeros(L, 1);
y1 = zeros(L, 1);
B2 = zeros(L, 1);
x2 = zeros(L, 1);
y2 = zeros(L, 1);

tiempo = zeros(L,1) ; 

%  Variable cara dolly
stateDolly = 'Frontal'; % 'esquina Izquierda'/'esquina Derecha'/'Lateral Izquierda'/'Lateral Derecha'

inicio=1;
staph=7400;

% figure()
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
    
    idx = 1:1:length(CYLINDER_ANGLES_i);
    idxLow = zeros(1,100);
    idxSup = zeros(1,100);
    
    % Si NO capta puntos de interes (se mantiene cilindro anterior)
    if (isempty(CYLINDER_ANGLES))
     
        CYLINDER = auxCylinder;
        CYLINDER_ANGLES = auxAngCylinder;
    
    else
        idx = 1:1:length(CYLINDER_ANGLES);
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
    auxMeanAng = mean(auxAngCylinder);

    % Calculo angulo Beta1
    [xR, yR] = pol2cart(auxMeanAng, min(auxCylinder)); 
    v_1 = [1,0,0] - [0,0,0];
    v_2 = [xR(1),yR(1),0] - [0.1,0,0];
    if yR(1) > 0
        ANGULO_B1 = atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    else
        ANGULO_B1 = -atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    end
    B1(i,1) = (ANGULO_B1);
    
    % Calculo coordenada x1,y1
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
    ptosCandidatosDolly = [x,y];

    %######################################################################
    % % G1
    % G1_ind = find(DOLLYANGBUENO > auxMeanAng+deg2rad(0.8));
    % G1 = DOLLYBUENO(G1_ind);  
    % G1_ang = DOLLYANGBUENO(G1_ind);
    % 
    % [xG1, yG1] = pol2cart(G1_ang, G1);
    % candidatosG1 = [xG1, yG1];
    % 
    % % Obtencion recta dolly G1
    % [mDollyG1, bDollyG1] = svdLineFit(candidatosG1);
    % y_predDollyG1 = bDollyG1 + mDollyG1*xG1; 
    % 
    % % Filtraje puntos G1 interes con Recta 
    % tol = 0.05;
    % errorDollyG1 = abs(y_predDollyG1 - candidatosG1(:,2));
    % idxG1 = find(errorDollyG1 <= tol); 
    % dollyPOINTSG1 = candidatosG1(idxG1,:);
    % 
    % % Puntos finales a coordenadas Polares G1
    % [dollyANG_G1, dollyRADIO_G1] = cart2pol(dollyPOINTSG1(:,1), dollyPOINTSG1(:,2));
    % 
    % %######################################################################
    % % G2
    % G2_ind = find(DOLLYANGBUENO < auxMeanAng - deg2rad(0.8));
    % G2 = DOLLYBUENO(G2_ind);  
    % G2_ang = DOLLYANGBUENO(G2_ind);
    % 
    % [xG2, yG2] = pol2cart(G2_ang, G2);
    % candidatosG2 = [xG2, yG2];
    % 
    % % Obtencion recta dolly G2
    % [mDollyG2, bDollyG2] = svdLineFit(candidatosG2);
    % y_predDollyG2 = bDollyG2 + mDollyG2*xG2; 
    % 
    % % Filtraje puntos G2 interes con Recta 
    % tol = 0.05;
    % errorDollyG2 = abs(y_predDollyG2 - candidatosG2(:,2));
    % idxG2 = find(errorDollyG2 <= tol); 
    % dollyPOINTSG2 = candidatosG2(idxG2,:);
    % 
    % % Puntos finales a coordenadas Polares G2
    % [dollyANG_G2, dollyRADIO_G2] = cart2pol(dollyPOINTSG2(:,1), dollyPOINTSG2(:,2));

    %######################################################################
    
%     if abs(mDollyG1 - mDollyG2) < 0.2 
%         mPromG = (mDollyG1 + mDollyG2)/2;
%         B2(i,1) = atan(-1/mPromG);
%     end
    
    
 
    % Angulo y radios de punto max y min de cilindro
    anguloFiltrajeCaras = 5;
    minanguloF = 1.2 ; 
    angSupDolly = DOLLYANGBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(0.8) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(anguloFiltrajeCaras));
    radSupDolly = DOLLYBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(0.8) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(anguloFiltrajeCaras));
    
    angInfDolly = DOLLYANGBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(0.8) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(anguloFiltrajeCaras));
    radInfDolly = DOLLYBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(0.8) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(anguloFiltrajeCaras));
    
    % Reconocimiento de Caras por radios
    if mean(radSupDolly)>0.85    &&  mean(radInfDolly)>0.85            
        stateDolly = 'Frontal';
          angSupDolly = DOLLYANGBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
            radSupDolly = DOLLYBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
    
        angInfDolly = DOLLYANGBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));
        radInfDolly = DOLLYBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));

    elseif mean(radSupDolly)<0.85    &&  mean(radInfDolly)<0.85
        if contains(stateDolly, 'esquina Derecha')
            stateDolly = 'Lateral Derecha';
            angSupDolly = DOLLYANGBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
            radSupDolly = DOLLYBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
        elseif contains(stateDolly, 'esquina Izquierda')
            stateDolly = 'Lateral Izquierda';
            angInfDolly = DOLLYANGBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));
            radInfDolly = DOLLYBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));        
        end
    elseif mean(radSupDolly)<0.85    &&  mean(radInfDolly)>0.85  
        stateDolly = 'esquina Derecha';
        angInfDolly = DOLLYANGBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));
        radInfDolly = DOLLYBUENO(DOLLYANGBUENO<min(CYLINDER_ANGLES)-deg2rad(minanguloF) & DOLLYANGBUENO>min(CYLINDER_ANGLES)-deg2rad(2*anguloFiltrajeCaras));
        
    elseif mean(radSupDolly)>0.85    &&  mean(radInfDolly)<0.85       
        stateDolly = 'esquina Izquierda';
        angSupDolly = DOLLYANGBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
        radSupDolly = DOLLYBUENO(DOLLYANGBUENO>max(CYLINDER_ANGLES)+deg2rad(minanguloF) & DOLLYANGBUENO<max(CYLINDER_ANGLES)+deg2rad(2*anguloFiltrajeCaras));
        
    else
        disp('FALLO')
    end
    disp(stateDolly)
    
    % Calculo de pendientes para theta2 segun Cara
    m = pendienteDolly(angSupDolly, radSupDolly, angInfDolly, radInfDolly, stateDolly); 
    B2(i,1) = atan(m);
    
    if i ~= 1 && (B2(i,1) - B2(i-1,1) > pi/2)   % Corrige cambios mayores a 90°
        B2(i,1) = -pi + B2(i,1);
    elseif i ~= 1 && (B2(i,1) - B2(i-1,1) < -pi/2)
        B2(i,1) = pi + B2(i,1);
    end


     % Calculo Coordenada x2,y2
    x2(i,1) = x1(i) + 0.78*cos(B2(i,1));
    y2(i,1) = y1(i) + 0.78*sin(B2(i,1));
    
    tiempo(i) = toc ; 
    
    %####### GRAFICOS #######%
    polarplot([0 0], [0 0.1], '-*')                 % Lidar
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')    % Puntos
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')      % Cilindro

%     polarplot(dollyANG_G1, dollyRADIO_G1, 'k.')           % Dolly G1
%     polarplot(dollyANG_G2, dollyRADIO_G2, 'y.')           % Dolly G2
    
    polarplot(angSupDolly, radSupDolly, 'm.')
    polarplot(angInfDolly, radInfDolly, 'g.')

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

figure         % Theta 2
plot(B2(:,1), '.')  
hold on
plot(theta(:,2), '.')
legend('\theta_2','ref')
title('Theta 2 (\theta_2)')
xlim([inicio staph])


%% Functions

function m = pendienteDolly(angSupDolly, radSupDolly, angInfDolly, radInfDolly, stateDolly) 
    [xSup, ySup] = pol2cart(angSupDolly, radSupDolly);
    [xInf, yInf] = pol2cart(angInfDolly, radInfDolly);
    
    if contains(stateDolly, 'Frontal') 
%         [m, ~] = svdLineFit([xSup , ySup ;  xInf , yInf]);
        m = polyfit([xSup ; xInf], [ySup ; yInf], 1);
        m = -1/m(1);
    elseif contains(stateDolly, 'Lateral Izquierda') || contains(stateDolly, 'Lateral Derecha')
%         [m, ~] = svdLineFit([xSup , ySup ;  xInf , yInf]);
        m = polyfit([xSup ; xInf], [ySup ; yInf], 1);
        m = m(1);
    elseif contains(stateDolly, 'esquina Izquierda')
%         [m, ~] = svdLineFit([xSup, ySup]);
        m = polyfit(xSup, ySup, 1);
        m = -1/m(1);
    elseif contains(stateDolly, 'esquina Derecha')
%         [m, ~] = svdLineFit([xInf, yInf]);
        m = polyfit(xInf, yInf, 1);
        m = -1/m(1);
    else
        m = 0;
    end

end

function [slope, intercept] = svdLineFit(points)
    % Suponemos que 'points' es una matriz nx2 de puntos [x, y]

    % Calcula la media de los puntos
    meanPoint = mean(points, 1);

    % Centraliza los puntos restando la media
    centeredPoints = points - meanPoint;

    % Aplica SVD a los puntos centralizados
    [U, S, V] = svd(centeredPoints);

    % La dirección de la línea que mejor se ajusta se corresponde con el 
    % primer vector singular derecho (la primera columna de V)
    direction = V(:, 1);

    % La pendiente de la línea es la relación entre las componentes y y x 
    % del vector de dirección
    slope = direction(2) / direction(1);

    % La intersección de la línea se puede calcular a partir de la pendiente 
    % y el punto medio
    intercept = meanPoint(2) - slope * meanPoint(1);
end
