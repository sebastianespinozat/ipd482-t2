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
for i=400:L


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
    
    [IDX, isnoise] = dbscan(Y, 0.05, 5);

    % modelClas = [1:1:length(unique(idx))-1; zeros(1, length(unique(idx))-1)];
    r2Model = [1:1:length(unique(IDX))-1; zeros(1, length(unique(IDX))-1)]; 
    % % Considerar r2, es mucho mas robusto c: y demuestra mas que el cluster
    % % representa a una recta.
    bestClusters = [0 0];

    for j=1:length(unique(IDX))-1
        jCluster = Y(IDX == j,:);
        jModel = fitlm(jCluster(:,1), jCluster(:,2));
        jCoeff = fliplr(jModel.Coefficients.Estimate');
%         modelClas(2,j) = jModel.RMSE;
        jR2 = jModel.Rsquared;
        r2Model(2,j) = jR2.Ordinary;
    end

    for k=1:2
        max_value = max(r2Model(2,:));
        [max_row, max_col] = find(r2Model == max_value, 1);
        bestClusters(k) = r2Model(1, max_col);
        r2Model(:, max_col) = [];
    end

    cluster1 = Y(IDX == bestClusters(1),:);
    cluster2 = Y(IDX == bestClusters(2),:);
    
    [tC1,rC1] = cart2pol(cluster1(:,1), cluster1(:,2));
    [tC2,rC2] = cart2pol(cluster2(:,1), cluster2(:,2));
    
    
    % graficos
    polarplot([0 0], [0 0.1], '-*')
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')
    hold on
    polarplot(tC1, rC1, 'm.')
    polarplot(tC2, rC2, 'c.')
    
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

