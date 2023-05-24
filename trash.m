clc;clear;close all;

%####### ANALISIS TIEMPO REAL LIDAR #########%
%####### DETECCION CILINDRO ######%
%####### DETECCION DOLLY POR CLASIFICACION DE GRUPOS ######%

% Carga de datos LIDAR
bag = rosbag('laser_data.bag');
% bsel = select(bag, 'Topic', '/scan');
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

% Ang y Coord Buscadas
B1 = zeros(L, 1);
x1 = zeros(L, 1);
y1 = zeros(L, 1);
B2 = zeros(L, 1);
x2 = zeros(L, 1);
y2 = zeros(L, 1);

inicio=5000;
staph=L;

figure()
for i=inicio:staph
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
    mA = auxAngCylinder(auxCylinder == min(auxCylinder));
%     [xR, yR] = pol2cart(mA, min(auxCylinder)); 
    [xR, yR] = pol2cart(auxMeanAng, min(auxCylinder)); 
    v_1 = [1,0,0] - [0,0,0];
    v_2 = [xR(1),yR(1),0] - [0.1,0,0];
    if yR(1) > 0
        ANGULO_B1 = atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    else
        ANGULO_B1 = -atan2(norm(cross(v_1, v_2)), dot(v_1, v_2));
    end
    
    B1(i,1) = (ANGULO_B1);
    
    
    
    %######## DOLLY ##########%
    % Filtrado por radio interes de DOLLY
    indicesDolly = find(SCANCOPY{i}.Ranges > 0.55 & SCANCOPY{i}.Ranges <= 1.5);
    DOLLY = SCANCOPY{i}.Ranges(indicesDolly);
    DOLLY_ANGLES = angulos(indicesDolly);

    % Filtrado por angulo de interes de DOLLY
    dollyANG_TOL = pi/6;
    indicesDolly = find(DOLLY_ANGLES < mean(CYLINDER_ANGLES) + dollyANG_TOL & DOLLY_ANGLES > mean(CYLINDER_ANGLES) - dollyANG_TOL);
    DOLLYBUENO = DOLLY(indicesDolly);
    DOLLYANGBUENO = DOLLY_ANGLES(indicesDolly);
    
    X = [DOLLYANGBUENO' DOLLYBUENO];
    [x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO');
    Y = [x', y'];
    
%     [IDX, isnoise] = dbscan(Y, 0.05, 5);
%     [IDX, isnoise] = dbscan(Y, 0.025, 15);
    % modelClas = [1:1:length(unique(idx))-1; zeros(1, length(unique(idx))-1)];
%     r2Model = [1:1:length(unique(IDX))-1; zeros(1, length(unique(IDX))-1)]; 
    % % Considerar r2, es mucho mas robusto c: y demuestra mas que el cluster
    % % representa a una recta.
%     bestClusters = [0 0];

    
%     for j=1:length(unique(IDX))-1
%         jCluster = Y(IDX == j,:);
%         jModel = fitlm(jCluster(:,1), jCluster(:,2));
%         jCoeff = fliplr(jModel.Coefficients.Estimate'); % m, p
% %       modelClas(2,j) = jModel.RMSE;
%         jR2 = jModel.Rsquared;
%         r2Model(2,j) = jR2.Ordinary;
%     end
% 
%     if length(unique(IDX)) >= 3
%         for k=1:2
%             max_value = max(r2Model(2,:));
%             [max_row, max_col] = find(r2Model == max_value, 1);
%             bestClusters(k) = r2Model(1, max_col);
%             r2Model(:, max_col) = [];
%         end
%         cluster1 = Y(IDX == bestClusters(1),:);
%         cluster2 = Y(IDX == bestClusters(2),:);
%         [tC1,rC1] = cart2pol(cluster1(:,1), cluster1(:,2));
%         [tC2,rC2] = cart2pol(cluster2(:,1), cluster2(:,2));
%         
%         mC1 = fitlm(cluster1(:,1), cluster1(:,2));
%         mC2 = fitlm(cluster2(:,1), cluster2(:,2));
% 
%         sprintf("m1: %f",mC1.Coefficients.Estimate(2))
%         sprintf("m2: %f",mC2.Coefficients.Estimate(2))
%         
%         
%         B2(i) = atan(-1/mean([mC1.Coefficients.Estimate(2) mC2.Coefficients.Estimate(2)])); 
%     else
%         cluster = Y;
%         mC1 = fitlm(cluster(:,1), cluster(:,2));
%         sprintf("m: %f",mC1.Coefficients.Estimate(2))
%         B2(i) = atan(-1/mC1.Coefficients.Estimate(2));
%         
%         [tC,rC] = cart2pol(Y(:,1), Y(:,2));
%  
%     end

  
    % G1
    G1_ind = find(DOLLYANGBUENO > auxMeanAng);
    G1 = DOLLYBUENO(G1_ind);  
    G1_ang = DOLLYANGBUENO(G1_ind);

    [xG1, yG1] = pol2cart(G1_ang, G1');
    YG1 = [xG1', yG1'];
    
    [IDX1, isnoise1] = dbscan(YG1, 0.025, 15);
    modelClas1 = [1:1:length(unique(IDX1))-1; zeros(1, length(unique(IDX1))-1)];
    r2Model1 = [1:1:length(unique(IDX1))-1; zeros(1, length(unique(IDX1))-1)]; 
    % Considerar r2, es mucho mas robusto c: y demuestra mas que el cluster
    % representa a una recta.
    bestCluster1 = [0];
    
    for j=1:length(unique(IDX1))-1
        jCluster1 = YG1(IDX1 == j,:);
        jModel1 = fitlm(jCluster1(:,1), jCluster1(:,2));
        jCoeff1 = fliplr(jModel1.Coefficients.Estimate'); % m, p
%       modelClas(2,j) = jModel.RMSE;
        j1R2 = jModel1.Rsquared;
        r2Model1(2,j) = j1R2.Ordinary;
    end

    if length(unique(IDX1)) >= 3    % Dos clusters + ruido
        max_value = max(r2Model1(2,:));
        [max_row, max_col] = find(r2Model1 == max_value, 1);
        bestCluster1 = r2Model1(1, max_col);        
        cluster1 = YG1(IDX1 == bestCluster1,:);
    else
        cluster1 = YG1;%(IDX1==1,:); 
    end
    [tC1,rC1] = cart2pol(cluster1(:,1), cluster1(:,2));
    mC1 = fitlm(cluster1(:,1), cluster1(:,2));
    sprintf("m: %f",mC1.Coefficients.Estimate(2))
    
    
    % G2
    G2_ind = find(DOLLYANGBUENO <= auxMeanAng);
    G2 = DOLLYBUENO(G2_ind);  
    G2_ang = DOLLYANGBUENO(G2_ind);


    [xG2, yG2] = pol2cart(G2_ang, G2');
    YG2 = [xG2', yG2'];
    
    [IDX2, isnoise2] = dbscan(YG2, 0.05, 10);
    modelClas2 = [1:1:length(unique(IDX2))-1; zeros(1, length(unique(IDX2))-1)];
    r2Model2 = [1:1:length(unique(IDX2))-1; zeros(1, length(unique(IDX2))-1)]; 
    % Considerar r2, es mucho mas robusto c: y demuestra mas que el cluster
    % representa a una recta.
    bestCluster2 = [0];
    
    for j=1:length(unique(IDX2))-1
        jCluster2 = YG2(IDX2 == j,:);
        jModel2 = fitlm(jCluster2(:,1), jCluster2(:,2));
        jCoeff2 = fliplr(jModel2.Coefficients.Estimate'); % m, p
%       modelClas(2,j) = jModel.RMSE;
        j2R2 = jModel2.Rsquared;
        r2Model2(2,j) = j2R2.Ordinary;
    end

    if length(unique(IDX2)) >= 3    % Dos clusters + ruido
        max_value = max(r2Model2(2,:));
        [max_row, max_col] = find(r2Model2 == max_value, 1);
        bestCluster2 = r2Model2(1, max_col); 
        cluster2 = YG2(IDX2 == bestCluster2,:);        
    else
        cluster2 = YG2;%(IDX2==1,:);
    end
    mC2 = fitlm(cluster2(:,1), cluster2(:,2));
    sprintf("m: %f",mC2.Coefficients.Estimate(2))
    [tC2,rC2] = cart2pol(cluster2(:,1), cluster2(:,2));


    
    % Graficos
    polarplot([0 0], [0 0.1], '-*')
    hold on
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')

    polarplot(G1_ang, G1, '.');
    polarplot(G2_ang, G2, '.');
    
    polarplot(tC1, rC1, 'm.')
    polarplot(tC2, rC2, 'c.')
    
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)
    rlim([0 1.5])
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
% ERROR DE ESTIMACION Y REFERENCIA
% err1 = immse(B1(:,1), theta);
% e1 = abs(B1(:,1) - theta);

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

