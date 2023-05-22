clc;clear;close all;


bag = rosbag('laser_data.bag');
bsel = select(bag, 'Topic', '/scan');
scan = bag.readMessages('DataFormat', 'struct');
SCANCOPY = scan;
L = length(scan);
rLIM = 1.5;
angulos = linspace(scan{1}.AngleMin, scan{1}.AngleMax, length(scan{1}.Ranges));

% figure('Name','polar plot')
% % Mostrar completo
% for i=1:L/4
%     polarplot(0,0, '*');
%     hold on
%     polarplot(angulos, scan{i}.Ranges, '.')
%     rlim([0 rLIM])
%     pause(1e-5)
%     hold off
% end
% Mostrar completo en cartesiano
% figure('Name','plot')
% for i=1:L/4
%     plot(0, 0, '*')
%     hold on
%     [x, y] = pol2cart(angulos, (scan{i}.Ranges)');
%     plot(x,y, '*')
%     grid on
%     ylim([-rLIM rLIM])
%     xlim([-rLIM rLIM])
%     hold off
%     pause(1e-5)
% end

figure()
for i=1:3
    indicesCilindro = find(SCANCOPY{i}.Ranges >= 0.4123 & SCANCOPY{i}.Ranges <= 0.55);
%     indicesDolly = find(SCANCOPY{i}.Ranges > 0.55 & SCANCOPY{i}.Ranges <= 1.25);
    
    CYLINDER = SCANCOPY{i}.Ranges(indicesCilindro);
    CYLINDER_ANGLES = angulos(indicesCilindro)';
    meanCylinder = mean(CYLINDER_ANGLES);

%     disp(std(CYLINDER_ANGLES))
% %     disp("PROMEDIO: ")
% %     disp(mean(CYLINDER))
%     if(std(CYLINDER_ANGLES)) > 0.02
%         step = deg2rad(15);
%         maxAng = max(CYLINDER_ANGLES);
%         minAng = min(CYLINDER_ANGLES);
%         
%         for j=minAng:step:maxAng-step
%             idx = find(CYLINDER_ANGLES >= j & CYLINDER_ANGLES < j+step);
%             if (var(CYLINDER(idx)) < 1e-3)
%                 angREAL = mean(CYLINDER_ANGLES(idx));
%                 disp("PROMEDIO REAL: ")
%                 disp(mean(CYLINDER(idx)))
%             end
%         end
%     end
    
%     DOLLY = SCANCOPY{i}.Ranges(indicesDolly);
%     DOLLY_ANGLES = angulos(indicesDolly);
% 
%     dollyANG_TOL = pi/6;
%     indicesDollyDolly = find(DOLLY_ANGLES < meanCylinder + dollyANG_TOL & DOLLY_ANGLES > meanCylinder - dollyANG_TOL);
%     DOLLYBUENO = DOLLY(indicesDollyDolly);
%     DOLLYANGBUENO = DOLLY_ANGLES(indicesDollyDolly);
    
    polarplot([0 0], [0 0.1], '-*')
    hold on
%     polarplot([0 angREAL], [0 0.55], '-*')
    hold on
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)
    hold on
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)
    hold on
%     polarplot(DOLLYANGBUENO, DOLLYBUENO, '.')
    rlim([0 1])
    hold on
%    polarplot(CYLINDER_ANGLES, CYLINDER, '.')
%     disp(rad2deg(mean(CYLINDER_ANGLES)))
    pause(1e-4)
    hold off
end

% 

X = load('local_trailer_x.mat');
X = cell2mat(struct2cell(X));
Y = load('local_trailer_y.mat');
Y = cell2mat(struct2cell(Y));
theta = load('local_trailer_theta.mat');
theta = cell2mat(struct2cell(theta));
theta = (theta);
theta = theta(:,1);
% ERROR DE ESTIMACION Y REFERENCIA
% err1 = immse(B1(:,1), theta);
% e1 = abs(B1(:,1) - theta);

figure
subplot 321
plot(B1(:,1), '.')
hold on
plot(theta, '.')
legend('Theta 1','ref')
title('Theta 1')
xlim([0 1500])
subplot 322
plot(abs(B1(:,1)-theta),'.')
xlim([0 1500])
title("error angulos")
subplot 323
plot(x1, '.')
hold on
plot(X(:,1),'.')
legend('x1','ref')
xlim([0 1500])
title('x1')
subplot 324
plot(abs(X(:,1)-x1),'.')
title("error X")
subplot 325
subplot 324
xlim([0 1500])
subplot 325
plot(y1, '.')
hold on; plot(Y(:,1),'.')
legend('y1','ref')
xlim([0 1500])
title('y1')
subplot 326
plot(abs(Y(:,1)-y1),'.')
title("error Y")
xlim([0 1500])




LB = 7400;
axisLength = 0.62;
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
% 
% step = deg2rad(30);
% for j=minAng:step:maxAng-step
% idx = find(CYLINDER_ANGLES >= j & CYLINDER_ANGLES < j+step);
% disp("GRADOS")
% disp(rad2deg(j))
% disp(" Y ")
% disp(rad2deg(j+step))
% disp(immse(CYLINDER(idx), mean(CYLINDER(idx))*ones(length(CYLINDER(idx)), 1)))
% end





