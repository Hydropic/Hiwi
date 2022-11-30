close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================

    % velocity GRAD/SEC
    max_velocity = deg2rad([90,85,80,80, 80,100]);
    min_velocity = deg2rad([-90,-85,-80,-80,-80,-100]);

    % acceleration GRAD/SEC2
    max_acceleration = deg2rad([200,200,200,200,200,200]);
    min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);


% load("Zwischenstand_Vor_ZOpti.mat")
axesPointConfigs = transpose(deg2rad( ...
               [137.481617, -59.297525, 130.556768, -151.220838, 73.439216, 81.101841;...   %Pos 1
                94.954001, -57.704881, 125.445174, -174.649310, 67.827874, 87.975653;...   %Pos 2
                75.104598, -46.819505, 96.630512, -190.760405, 50.311301, 96.919871;...   %Pos 3
                69.041729, -38.632705, 77.211118, -202.266841, 40.759665, 107.230905]));     %Pos 4
% save("Zwischenstand_Vor_ZOpti.mat")
a = 0.05; 
timeSteps = [0.50449308069551+a ...	    %1
             0.68449308069551+a ...	    %2
             0.80449308069551+a];       %3

VelocityBoundaryCondition_x =       [1.11626424791510	1.11626424791510	0.0];
AccelerationBoundaryCondition_x =   [-1.31940823068824  -1.31940823068824	0.0];


VelocityBoundaryCondition_y =       [-1.18494663299643  -1.18494663299643	0.0];
AccelerationBoundaryCondition_y =   [1.48604587496325	1.48604587496325	0.0];

VelocityBoundaryCondition_xyz_middle_XY = [VelocityBoundaryCondition_x; VelocityBoundaryCondition_y];
AccelerationBoundaryCondition_xyz_middle_XY = [AccelerationBoundaryCondition_x; AccelerationBoundaryCondition_y];

% Aufbau Value:
            % Zeitpunkte
            % VelocityBoundaryCondition_x
            % VelocityBoundaryCondition_y
            % AccelerationBoundaryCondition_x
            % AccelerationBoundaryCondition_y
timeMax = 1.0;
timeMin = 0.4;
vel = 2.1;
acc_x_TCP = 4.5;
acc_y_TCP = 3.5;

max_values = [1.0 0.8 1.1;... 
              acc_y_TCP acc_y_TCP 0.0;... 
              acc_x_TCP acc_x_TCP 0.0;...  
              vel vel 0.0;... 
              vel vel 0.0];

min_values = [timeMin timeMin timeMin;... 
              -acc_y_TCP -acc_y_TCP 0.0;...
              -acc_x_TCP -acc_x_TCP 0.0;... 
              -vel -vel 0.0;...
              -vel -vel 0.0];

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 0;
    bool_skp_Optimirung_Kat = 1;
    splineDiscretization = 40;
    maxIterationsSplineTCP = 70;
    visualizeTCPPath = 1;
    saveEMI = 0;
    % Set up Boundarys für first Simulation
    jerkBoundaries = 0.4; % Für die Ruckänderung an den Mittelpunkten gilt +- dieser Wert als Max. bzw. Min.
    optimalSplineDiscretization = 80;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%ZRot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    offsetToBorder = 5; %Z.B 5
    minIter = 120;%120
    maxiter = 2000;%2000
    span_to_Smooth = 0.05;%0.025 %As value from 0 to 1  !!!!MAX 0.1 SONST FEHLERANFÄLLIG!!!!!
    stepsize = 10;%10
    widhtStuetzp = 5; % Grade Zahl z.B 4 oder 0
    grenzSchwap_Y = 2.5;%STANDART 2.5!!!!

optiZ_Rot_Param = [offsetToBorder,minIter,maxiter,span_to_Smooth,stepsize,widhtStuetzp,grenzSchwap_Y];
optiZ_Rot_Param2 = optiZ_Rot_Param;
    
booleanManualPostProcessing = 0;

booleanTimeOpimizationTure = 0;
    maxIterations = 20;
    timeStepSize = 0.06; % nicht unter 0.05

booleanSloshingKompensationTrans = 0;
    numOfIterations = 70;

booleanSloshingKompensationRot = 0;

booleanVisualisation = 1;




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

%% ======== Optimierung: Beschl.-Profil TCP u. Erzeugung: Base-Points =====
if booleanFormTCP
    if bool_skp_Optimirung_Kat == 0
            
        % Initiale Werte für die Optimierung setzen
        init_ax_values_YX = [timeSteps; VelocityBoundaryCondition_xyz_middle_XY; AccelerationBoundaryCondition_xyz_middle_XY];  
        save("Zwischenstand_Vor_ZOpti.mat")
        load("Zwischenstand_Vor_ZOpti.mat")
    
        % Visualisieren der TCP-Bahn u. Speichern der Ergebnisse im EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, init_ax_values_YX, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries);
        for t =1:1
        % Zeitinervalle und Position der Kollisionspunkte optimieren
        [x_xy, optiResuls] = splineOptimization(optimalSplineDiscretization, maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values_YX);
    

        % Sichern aller Parameter für die nächte Optimierung
        init_ax_values_YX = [x_xy(:,:)]
        save("Zwischenstand_Vor_ZOpti.mat")

        % Speichern der Ergebnisse in EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xy, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries);
        end
        % Profil in Z Richtung identifizieren
        VelocityBoundaryCondition_z =       [0   0   0.0];
        AccelerationBoundaryCondition_z =   [0	0	0.0];
        
        VelocityBoundaryCondition_xyz_middle_Z = VelocityBoundaryCondition_z;
        AccelerationBoundaryCondition_xyz_middle_Z = AccelerationBoundaryCondition_z;

        min_values_Z = [x_xy(1,:); 
                  -5.0 -5.0 0.0; 
                  -5.0 -5.0 0.0];
    
        max_values_Z = [x_xy(1,:); 
                      5.0 5.0 0.0;  
                      5.0 5.0 0.0];
    
        % Initiale Werte für die Optimierung setzen
        init_ax_values_Z = [x_xy(1,:); VelocityBoundaryCondition_xyz_middle_Z; AccelerationBoundaryCondition_xyz_middle_Z];   
    
        %load("Zwischenstand_Vor_ZOpti.mat")
        [x_z, optiResuls] = splineOptimization_z(optimalSplineDiscretization, 3, splineDiscretization, axesPointConfigs, min_values_Z, max_values_Z, jerkBoundaries, init_ax_values_Z);
    
        x_xyz(1,:) = x_xy(1,:);
        x_xyz(2,:) = x_xy(2,:);
        x_xyz(3,:) = x_xy(3,:);
        x_xyz(4,:) = x_z(2,:);
        x_xyz(5,:) = x_xy(4,:);
        x_xyz(6,:) = x_xy(5,:);
        x_xyz(7,:) = x_z(3,:);
    
        % Speichern der Ergebnisse in EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xyz, splineDiscretization, axesPointConfigs, min_values_Z, max_values_Z, jerkBoundaries);
        %Überschreiben der Zwischenergebnissen

        % Sichern aller Parameter für die nächte Optimierung
        init_ax_values_Z = [x_z(:,:)]
        save("Zwischenstand_Vor_ZOpti.mat")
    else
        load("Zwischenstand_Vor_ZOpti.mat")

        % Speichern der Ergebnisse in EMI Format
        [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xyz, splineDiscretization, axesPointConfigs, min_values_Z, max_values_Z, jerkBoundaries);
    end
    
    save("Zwischenstand_Vor_ZOpti.mat")
    %Z-Rot optimierung    
    [eulerZYX,acc_XYZ] = generate_Z_Rot(x_xyz,axesPointConfigs,splineDiscretization,visualizeTCPPath,optiZ_Rot_Param2);

    % load("Zwischenstand_Vor_ZOpti.mat")
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
    
        save('SimResults_A6.mat','x','-v7.3');

        generierenEmily(optimized_translational_values_load);
end    

%% ======== CFD ===========================================================
% TODO: Impemetieren
