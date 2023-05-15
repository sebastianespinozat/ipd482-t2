clc;clear;close all;


bag = rosbag('laser_data.bag');
% bsel = select(bag, 'Topic', '/scan');
scan = bag.readMessages('DataFormat', 'struct');
SCANCOPY = scan;
L = length(scan);
rLIM = 1.5;
angulos = linspace(scan{1}.AngleMin, scan{1}.AngleMax, length(scan{1}.Ranges));


MAX_cylindro = deg2rad(80);%tan(0.4/0.1);
auxCylinder = zeros(1,1);
auxAngCylinder = zeros(1,1);
justStarted = 1;


figure()
for i=1:L/10


    sprintf("Iteracion: %d", i)
    indicesCilindro = find(SCANCOPY{i}.Ranges >= 0.4123 & SCANCOPY{i}.Ranges <= 0.55);
    
    CYLINDER = SCANCOPY{i}.Ranges(indicesCilindro);
    
    CYLINDER_ANGLES = angulos(indicesCilindro)';
    
    indicesAngCilindro = find(CYLINDER_ANGLES <= MAX_cylindro & CYLINDER_ANGLES >= -MAX_cylindro);
    
    % CYLINDER_ANGLES y CYLINDER se sobreescriben en cada iteraci�n para
    % calculos de ECM
    CYLINDER_ANGLES = CYLINDER_ANGLES(indicesAngCilindro);
    CYLINDER = CYLINDER(indicesAngCilindro);
    
    CYLINDER_ANGLES_i = CYLINDER_ANGLES;
    CYLINDER_i = CYLINDER;
    
%     sprintf("CANT PUNTOS: %d", length(CYLINDER_ANGLES))
%     sprintf("CANT PUNTOS AUX: %d", length(auxAngCylinder))
    maxAng = max(CYLINDER_ANGLES_i);
    minAng = min(CYLINDER_ANGLES_i);
    lowA = minAng;
    supA = maxAng;
    ECM = immse(CYLINDER_ANGLES_i, mean(CYLINDER_ANGLES_i).*ones(1,length(CYLINDER_ANGLES_i))');
    
    
    idx = 1:1:length(CYLINDER_ANGLES_i);
    idxLow = zeros(1,100);
    idxSup = zeros(1,100);
    
    
    
    if (isempty(CYLINDER_ANGLES))
        
        CYLINDER = auxCylinder;
        CYLINDER_ANGLES = auxAngCylinder;
    
    else
        idx = 1:1:length(CYLINDER_ANGLES);
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
        CYLINDER_ANGLES = CYLINDER_ANGLES_i(idx);
        CYLINDER = CYLINDER_i(idx);
        
        if (justStarted == 1)
            auxMeanAng = mean(CYLINDER_ANGLES);
            justStarted = 0;
        end
        if (abs(mean(CYLINDER_ANGLES) - auxMeanAng) > deg2rad(10) || length(CYLINDER_ANGLES) < 6)
            CYLINDER = auxCylinder;
            CYLINDER_ANGLES = auxAngCylinder;
        end
    end

    auxCylinder = CYLINDER;
    auxAngCylinder = CYLINDER_ANGLES;
    auxMeanAng = mean(auxAngCylinder);

    % DOLLY 
    indicesDolly = find(SCANCOPY{i}.Ranges > 0.55 & SCANCOPY{i}.Ranges <= 1.5);
    DOLLY = SCANCOPY{i}.Ranges(indicesDolly);
    DOLLY_ANGLES = angulos(indicesDolly);

    dollyANG_TOL = pi/4;
    indicesDolly = find(DOLLY_ANGLES < mean(CYLINDER_ANGLES) + dollyANG_TOL & DOLLY_ANGLES > mean(CYLINDER_ANGLES) - dollyANG_TOL);
    DOLLYBUENO = DOLLY(indicesDolly);
    DOLLYANGBUENO = DOLLY_ANGLES(indicesDolly);
    
    X = [DOLLYANGBUENO' DOLLYBUENO];
    [x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO');
    Y = [x', y'];
    
    [IDX, isnoise] = dbscan(Y, 0.1, 10);
    bestCluster = 100;
    for k=1:length(unique(IDX))-1
        iCluster = Y(IDX == k,:);
        iModel = fitlm(iCluster(:,1), iCluster(:,2));
        iCoeff = iModel.Coefficients.Estimate;
        yi_pred = predict(iModel, iCluster(:,1));
        iECM = immse(iCluster(:,2), yi_pred);

        if iECM <= bestCluster
            bestCluster = k;
        end

    end


    straightLine = Y(IDX == bestCluster,:);
    [thetita,rho] = cart2pol(straightLine(:,1), straightLine(:,2));


    % graficos
    
    polarplot([0 0], [0 0.1], '-*')
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')
  
    polarplot(thetita, rho, 'g.')
    
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)
    rlim([0 1.5])
    pause(1e-5)
    hold off
    
end


% [x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO');
%     
% % Concatenar los vectores de datos
% data = [DOLLYANGBUENO', DOLLYBUENO];
% 
% % Realizar k-means clustering con k = 3
% [idx, C] = kmeans(data, 3);
% 
% % Visualizar los grupos obtenidos
% figure
% scatter(x, y, 10, idx, 'filled')










%% Comprobacion datos

% X = load('local_trailer_x.mat');
% X = cell2mat(struct2cell(X));
% Y = load('local_trailer_y.mat');
% Y = cell2mat(struct2cell(Y));
% theta = load('local_trailer_theta.mat');
% theta = cell2mat(struct2cell(theta));
% LB = 7400;
% axisLength = 0.62;
% 
% figure('name','palo')
% for i=1:2000
%     [x11, y11, x12, y12] = axisPos(X(i,1),Y(i,1),theta(i,1), axisLength/2);
%     [x21, y21, x22, y22] = axisPos(X(i,2),Y(i,2),theta(i,2), axisLength/2);
%     plot([x11 x12], [y11 y12], '-*', 'MarkerSize',10)
%     hold on
%     plot([x21 x22], [y21 y22], '-*','MarkerSize',10)
%     hold on
%     plot([X(i,1) X(i,2)], [Y(i,1) Y(i,2)], '-*','MarkerSize',10)
%     pause(1e-5)
%     hold off
%     xlim([-4 4])
%     ylim([-4 4])
%     legend('front axis','rear axis', 'core axis')
% end
% 
% 
% function [x1, y1, x2, y2] = axisPos(x,y,theta, L)
%     x1 = x - L*sin(theta);
%     y1 = y - L*cos(theta);
%     
%     x2 = x + L*sin(theta);
%     y2 = y + L*cos(theta);
% end
% 

