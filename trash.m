clc;clear;close all;


bag = rosbag('laser_data.bag');
% bsel = select(bag, 'Topic', '/scan');
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

MAX_cylindro = deg2rad(80);%tan(0.4/0.1);
auxCylinder = zeros(1,1);
auxAngCylinder = zeros(1,1);
justStarted = 1;


figure()
for i=7400:L


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
%     sprintf("L: %d", length(CYLINDER_ANGLES))
    % agregar IF del g
%     sprintf("idx: %d", length(idx))
    auxCylinder = CYLINDER;
    auxAngCylinder = CYLINDER_ANGLES;
    auxMeanAng = mean(auxAngCylinder);

    polarplot([0 0], [0 0.1], '-*')
    hold on
%     polarplot(CYLINDER_ANGLES, CYLINDER, '.')
    polarplot(angulos, SCANCOPY{i}.Ranges, 'b.')
    hold on
    polarplot(CYLINDER_ANGLES, CYLINDER, 'r.')
%     hold on
    polarplot(linspace(0,2*pi,50),ones(50)*0.413)
    hold on
    polarplot(linspace(0,2*pi,50),ones(50)*0.55)
    hold on
    rlim([0 1.5])
    hold on
%    polarplot(CYLINDER_ANGLES, CYLINDER, '.')
    pause(1e-5)
    hold off

    
end



  
%         if (abs(lowA-supA) < 0.13)  % Toma todos los indices (solo esta el cilindro)
%             idx = 1:1:length(CYLINDER_ANGLES);
%         else % Hay mas angulos
%             while (ECM > 1e-3)%
%                 
%                 splitAng = mean([lowA supA]);
%                 diffAng = abs(lowA - supA);
% 
%                 idxLow = find(CYLINDER_ANGLES_i < splitAng);
%                 idxSup = find(CYLINDER_ANGLES_i >= splitAng);
% 
%                 ecmL = immse(CYLINDER_ANGLES_i(idxLow), mean(CYLINDER_ANGLES_i(idxLow)*ones(1,length(CYLINDER_ANGLES_i(idxLow))))');
%                 ecmS = immse(CYLINDER_ANGLES_i(idxSup), mean(CYLINDER_ANGLES_i(idxSup)*ones(1,length(CYLINDER_ANGLES_i(idxSup))))');    
%                 if ecmL < ecmS 
%                     ECM = ecmL;
%                     lowA = splitAng - diffAng/2;
%                     supA = splitAng;
%                     idx = idxLow;
%                 else
%                     ECM = ecmS;
%                     lowA = splitAng;
%                     supA = splitAng + diffAng/2;
%                     idx = idxSup;
%                 end
%                 % Se sobreescribe en cada iteracion del while c:
%                 
%             end
%         end  
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
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

