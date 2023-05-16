clear;close all;clc

load('dolly.mat')
load('ang_dolly.mat')




figure
polarplot(DOLLYANGBUENO, DOLLYBUENO, '.')


[x, y] = pol2cart(DOLLYANGBUENO, DOLLYBUENO');
figure
plot(x,y, '.')


X = [DOLLYANGBUENO' DOLLYBUENO];
Y = [x', y'];

[idx, isnoise] = dbscan(Y, 0.05, 5);

figure;
xlim([0 1.5])
ylim([-0.5 1])
scatter(Y(:,1), Y(:,2), 30, idx, 'filled');
title('DBSCAN Clustering');
xlabel('X');
ylabel('Y');
hold on


modelClas = [1:1:length(unique(idx))-1; zeros(1, length(unique(idx))-1)];
r2Model = [1:1:length(unique(idx))-1; zeros(1, length(unique(idx))-1)]; 
% Considerar r2, es mucho mas robusto c: y demuestra mas que el cluster
% representa a una recta.
bestClusters = [0 0];

% pause(5)
% figure
for i=1:length(unique(idx))-1
    disp("-------------")
    iCluster = Y(idx == i,:);
    iModel = fitlm(iCluster(:,1), iCluster(:,2));
    iCoeff = fliplr(iModel.Coefficients.Estimate');
    disp(iModel)
    yi_pred = predict(iModel, iCluster(:,1));

    y_pred = polyval(iCoeff, iCluster(:,1));
    iECM = iModel.RMSE;
    modelClas(2,i) = iECM;
    iR2 = iModel.Rsquared;
    r2Model(2,i) = iR2.Ordinary;

    plot(iCluster(:,1), iCluster(:,2), '.');
    hold on 
    plot(iCluster(:,1), yi_pred, 'r-', 'LineWidth', 2);
end


% Obtencion de los cluster que mejor representa a las rectas (2)
disp("RMSE")
disp(modelClas)
disp("R-squared")
disp(r2Model)

for j=1:2
    max_value = max(r2Model(2,:));
    [max_row, max_col] = find(r2Model == max_value, 1);
    bestClusters(j) = r2Model(1, max_col);
    r2Model(:, max_col) = [];

end

disp("R-squared elimination")
disp(r2Model)

disp("Best clusters")
disp(bestClusters)

cluster1 = Y(idx == bestClusters(1),:);
cluster2 = Y(idx == bestClusters(2),:);

figure
plot(cluster1(:,1), cluster1(:,2), '.');
hold on
plot(cluster2(:,1), cluster2(:,2), '.');
xlim([0 1.5])
ylim([-0.5 1])


[tC1,rC1] = cart2pol(cluster1(:,1), cluster1(:,2));
[tC2,rC2] = cart2pol(cluster2(:,1), cluster2(:,2));
figure
polarplot([0 0], [0 0.1], '-*')
hold on
polarplot(DOLLYANGBUENO, DOLLYBUENO, '.')
polarplot(tC1, rC1, 'm.')
polarplot(tC2, rC2, 'c.')

polarplot(linspace(0,2*pi,50),ones(50)*0.413)
polarplot(linspace(0,2*pi,50),ones(50)*0.55)
rlim([0 1.5])
    

% straightLine = Y(idx == bestCluster,:);
% [thetita,rho] = cart2pol(straightLine(:,1), straightLine(:,2));
% 
% figure
% polarplot(thetita,rho, '.')
% hold on
% plot(straightLine(:,1), straightLine(:,2), '.');

% for j in each cylinder border (2)
% for i in each cluster
% if recta in cluster_i -> tomar cluster de la linea



