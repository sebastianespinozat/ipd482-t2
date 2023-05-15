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
% [IDX,C] = kmeans(X,2);
% figure;
% scatter(x, y, [], IDX, 'filled');
% % hold on;
% % scatter(C(:,1), C(:,2), 100, 'k', 'filled');
% % legend('Clúster 1', 'Clúster 2', 'Centroides');
% xlabel('X');
% ylabel('Y');
% 
% NICE = X(IDX == 1,:);
% notNICE = X(IDX ~= 1,:);
% 
% [thetaNICE,rhoNICE] = cart2pol(NICE(:,1), NICE(:,2));
% [thetaNNICE,rhoNNICE] = cart2pol(notNICE(:,1), notNICE(:,2));
% 
% figure
% polarplot(thetaNICE, rhoNICE, '.')
% hold on
% polarplot(thetaNNICE, rhoNNICE, '.')
% plot(NICE(:,1), NICE(:,2), '.')
% hold on
% plot(notNICE(:,1), notNICE(:,2), '.')
% legend('nice', 'notNICE')


[idx, isnoise] = dbscan(Y, 0.1, 10);
bestCluster = 100;
figure;
scatter(Y(:,1), Y(:,2), 30, idx, 'filled');
title('DBSCAN Clustering');
xlabel('X');
ylabel('Y');
hold on
% figure
for i=1:length(unique(idx))-1
    iCluster = Y(idx == i,:);
    iModel = fitlm(iCluster(:,1), iCluster(:,2));
    iCoeff = iModel.Coefficients.Estimate;
    yi_pred = predict(iModel, iCluster(:,1));
    iECM = immse(iCluster(:,2), yi_pred);

    if iECM <= bestCluster
        bestCluster = i;
    end
    disp(iECM)
    plot(iCluster(:,1), iCluster(:,2), '.');
    pause(2)
end


straightLine = Y(idx == bestCluster,:);
[thetita,rho] = cart2pol(straightLine(:,1), straightLine(:,2));

% figure
% polarplot(thetita,rho, '.')
% hold on
% plot(straightLine(:,1), straightLine(:,2), '.');





