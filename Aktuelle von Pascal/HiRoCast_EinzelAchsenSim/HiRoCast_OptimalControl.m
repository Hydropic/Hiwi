close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================
axesPointConfigs = transpose(deg2rad( ...
               [137.465165, -49.530261, 100.796164, -130.441749, 62.511117, 61.562215;...   %Axe 1
                120.083395, -72.750982, 119.697534, -126.076824, 61.182276, 56.512061;...   %Axe 2
                95.924290, -62.667657, 136.521858, -163.456914, 74.489726, 85.458555;...    %Axe 3
                96.351437, -51.944629, 106.677805, -154.415388, 57.468990, 75.561543;...    %Axe 4
                92.164042, -51.676624, 106.018066, -153.372486, 57.324680, 74.854384;...    %Axe 5
                65.272690, -53.641945, 110.904103, -173.738885, 57.417892, 86.618787;...    %Axe 6
                60.368131, -59.864982, 127.579923, -132.411512, 74.548830, 73.740877;...    %Axe 7
                59.388612, -61.607954, 132.929665, -119.270830, 80.614438, 73.777782;...    %Axe 8
                72.844770, -22.588701, 65.998502, -101.977124, 77.626892, 44.712782]));     %Axe 9

a = 0.05; 
timeSteps = [0.74449308069551+a ...	    %1
             0.49449308069551+a ...	    %2	
             0.59449308069551+a ...	    %3
             0.40449308069551+a ...	    %4
             0.39449308069551+a ...	    %5
             0.89449308069551+a ...	    %6
             0.829449308069551+a ...    %7	
             0.6049308069551+a];        %8

VelocityBoundaryCondition_x =       [1.89324092806605	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	0.0];
AccelerationBoundaryCondition_x =   [1.32791857795363	-3.31940823068824	-3.31940823068824	-3.31940823068824	-3.31940823068824	-3.31940823068824	-3.31940823068824	0.0];


VelocityBoundaryCondition_y =       [-1.95367961344194	-1.18494663299643	-1.18494663299643	-1.18494663299643	-1.18494663299643	-1.18494663299643	-1.18494663299643	0.0];
AccelerationBoundaryCondition_y =   [-2.90067458258856	4.48604587496325	4.48604587496325	4.48604587496325	4.48604587496325	4.48604587496325	4.48604587496325	0.0];

VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x; VelocityBoundaryCondition_y];
AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x; AccelerationBoundaryCondition_y];

% Aufbau Value:
            % Zeitpunkte
            % VelocityBoundaryCondition_x
            % VelocityBoundaryCondition_y
            % AccelerationBoundaryCondition_x
            % AccelerationBoundaryCondition_y
timeMax = 2.0;
timeMin = 0.3;
acc = 5;
vel = 5;

max_values = [timeMax timeMax timeMax timeMax timeMax timeMax timeMax timeMax;... 
              vel vel vel vel vel vel vel 0.0;... 
              vel vel vel vel vel vel vel 0.0;...  
              acc acc acc acc acc acc acc 0.0;... 
              acc acc acc acc acc acc acc 0.0];

min_values = [timeMin timeMin timeMin timeMin timeMin timeMin timeMin timeMin;... 
              -vel -vel -vel -vel -vel -vel -vel 0.0;...
              -vel -vel -vel -vel -vel -vel -vel 0.0;... 
              -acc -acc -acc -acc -acc -acc -acc 0.0;...
              -acc -acc -acc -acc -acc -acc -acc 0.0];

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 1;
splineDiscretization = 40;
maxIterationsSplineTCP = 40;
visualizeTCPPath = 1;
saveEMI = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%ZRot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
offsetToBorder = 5; %Z.B 5
minIter = 120;%120
maxiter = 2000;%2000
span_to_Smooth = 0.05;%0.025 %As value from 0 to 1  !!!!MAX 0.1 SONST FEHLERANFÄLLIG!!!!!
stepsize = 10;%10
widhtStuetzp = 5; % Grade Zahl z.B 4 oder 0
grenzSchwap_Y = 2.5;%STANDART 2.5!!!!

optiZ_Rot_Param = [offsetToBorder,minIter,maxiter,span_to_Smooth,stepsize,widhtStuetzp,grenzSchwap_Y];
optiZ_Rot_Param1 = optiZ_Rot_Param;
    
booleanManualPostProcessing = 1;

booleanTimeOpimizationTure = 0;
    maxIterations = 20;
    timeStepSize = 0.06; % nicht unter 0.05

booleanSloshingKompensationTrans = 1;
    numOfIterations = 0;

booleanSloshingKompensationRot = 0;

booleanVisualisation = 0;

bool_skp_Optimirung_Kat = 1;

%% ======== Optimierung: Beschl.-Profil TCP u. Erzeugung: Base-Points =====
if booleanFormTCP
    if bool_skp_Optimirung_Kat == 0
        % Set up Boundarys für first Simulation
        jerkBoundaries = 0.5; % Für die Ruckänderung an den Mittelpunkten gilt +- dieser Wert als Max. bzw. Min.
        optimalSplineDiscretization = 80;
    
        % Initiale Werte für die Optimierung setzen
        init_ax_values = [timeSteps; VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   
    
        % Visualisieren der TCP-Bahn u. Speichern der Ergebnisse im EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, init_ax_values, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries);
        for t =1:1
        % Zeitinervalle und Position der Kollisionspunkte optimieren
        [x_xy, optiResuls] = splineOptimization(optimalSplineDiscretization, maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values);
    
        % Speichern der Ergebnisse in EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xy, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries);
        init_ax_values = x_xy;
        end
        % Profil in Z Richtung identifizieren
        VelocityBoundaryCondition_x =       [1.89324092806605	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	0.0];
        AccelerationBoundaryCondition_x =   [1.89324092806605	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	0.0];
        
        VelocityBoundaryCondition_xyz_middle = VelocityBoundaryCondition_x;
        AccelerationBoundaryCondition_xyz_middle = AccelerationBoundaryCondition_x;
    
        min_values = [x_xy(1,:); 
                  -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 0.0; 
                  -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 0.0];
    
        max_values = [x_xy(1,:); 
                      4.5 4.5 4.5 4.5 4.5 4.5 4.5 0.0;  
                      4.5 4.5 4.5 4.5 4.5 4.5 4.5 0.0];
    
        % Initiale Werte für die Optimierung setzen
        init_ax_values = [x_xy(1,:); VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   
    
        [x_z, optiResuls] = splineOptimization_z(optimalSplineDiscretization, maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values);
    
        x_xyz(1,:) = x_xy(1,:);
        x_xyz(2,:) = x_xy(2,:);
        x_xyz(3,:) = x_xy(3,:);
        x_xyz(4,:) = x_z(2,:);
        x_xyz(5,:) = x_xy(4,:);
        x_xyz(6,:) = x_xy(5,:);
        x_xyz(7,:) = x_z(3,:);
    
        % Speichern der Ergebnisse in EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xyz, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries);
        %Überschreiben der Zwischenergebnissen
        save("Zwischenstand_Vor_ZOpti.mat")
    else
        load("Zwischenstand_Vor_ZOpti.mat")
    end
    
    %Z-Rot optimierung
    [eulerZYX,acc_XYZ] = generate_Z_Rot(x_xyz,axesPointConfigs,splineDiscretization,visualizeTCPPath,optiZ_Rot_Param1);

    % Generieren aller Bais-Points
    [x] = backwardTransformationRoboDK(Position_xyz, timeLine, splineDiscretization, x_xyz, axesPointConfigs,eulerZYX);
    
    save('SimResults.mat','x','-v7.3');
end

%% ======== Visualisierung u. Händische Nachbearbeitung ===================
if booleanManualPostProcessing
    [test] = ManualPostProcessing(splineDiscretization, visualizeTCPPath,max_values,min_values,jerkBoundaries);
end

%% ======== Optimierung: Achse 1 - 5 ======================================
if booleanTimeOpimizationTure
    global axis;
    global seriesSim;
    global fixedTime;
    axis = axisNum();
    optiResuls123 = [];
    % check most slove Axis
    for axis = 3:3
        fixedTime = false         
        [x, optiResuls] = timeOptimization(timeStepSize, maxIterations, false, startConfig, goalConfig, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition);
        show_spline(x, 'Bad Output');
        optiResuls123(axis, :) = optiResuls;
        save('SimResults.mat','x','-v7.3');
    end
    [minTime, minIndex] = max(optiResuls123(:, 1))    
    
    if minIndex == 1
        seriesSim = [1, 2, 3];
    elseif minIndex == 2
        seriesSim = [2, 1, 3];
    elseif minIndex == 3
        seriesSim = [3, 1, 2];
    else
    end

    % optimize frome slove to fast
    for p = 1:3
        axis = p;
        if axis == 1
            fixedTime = false;
        else
            fixedTime = true;
            maxIterations = 30;
        end
        [x, optiResuls] = timeOptimization(timeStepSize, maxIterations, true, startConfig, goalConfig, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition);
        show_spline(x, 'Bad Output');
        save('SimResults.mat','x','-v7.3');
    end
end

%% ======== Optimierung: Achse 6 ==========================================
if booleanSloshingKompensationTrans
        [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations);
        
        example = matfile('SimResults.mat');
        optimized_translational_values_load = example.x;
        
        optimized_translational_values_load(:,7) = x(:,1)

        x = optimized_translational_values_load;
    
        save('SimResults.mat','x','-v7.3');
end    

%% ======== CFD ===========================================================
% TODO: Impemetieren

%% ====== Visualisierung ==================================================
if booleanVisualisation
    example = matfile('SimResults.mat');
    optimized_translational_values = example.x;
    minJerkPath = optimized_translational_values(:, 2:7)
    timeSum = sum(optimized_translational_values(:, 1));

    show_spline(optimized_translational_values, 'Bad Output');

%     KSetUp;
%     showBahn = optimized_translational_values;
%     showBahn(:,1) = [];
%     laufbahn(robot,showBahn,1,true)    
       
    for z = 1:length(optimized_translational_values)
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(optimized_translational_values(z,2:7));
        optimized_translational_values(z,6) = eulerZYX(2);
        optimized_translational_values(z,7) = eulerZYX(3);
    end
    show_spline(optimized_translational_values, 'y (um Achse 6), x (um Achse 5)');

    generierenEmily(optimized_translational_values);
    % save('SimResults.mat','x','-v7.3');
end 