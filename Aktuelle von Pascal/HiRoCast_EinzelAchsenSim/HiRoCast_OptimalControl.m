close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================
axesPointConfigs = transpose(deg2rad( ...
               [137.113917, -50.020898, 95.469279, -123.499925, 61.481928, 54.195993;
                114.610471, -76.854244, 119.815419, -146.093857, 48.291774, 65.906197;
                80.868859, -66.045040, 132.379939, -189.953423, 66.652709, 93.978336;
                82.410186, -59.431507, 116.722884, -188.998553, 57.613160, 94.848339; 
                77.608274, -58.491769, 114.564882, -182.881707, 56.106680, 91.607913;
                78.929890, -52.725884, 101.514595, -138.195382, 56.862399, 63.948663;
                85.880409, -51.738553, 99.303763, -96.756601, 83.860758, 47.927961;
                75.419236, -68.607998, 138.858956, -148.042386, 73.059001, 79.697323;
                62.074593, -34.121021, 104.137621, -209.422911, 72.424992, 99.664834]));

timeSteps = [0.69449308069551 ...	
             0.69449308069551 ...		
             0.89449308069551 ...	
             0.89449308069551 ...	
             0.89449308069551 ...	
             0.99449308069551 ...	
             0.979449308069551 ...	
             1.29449308069551];

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


acc = 5;
vel = 5;
min_values = [0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3;... 
              -vel -vel -vel -vel -vel -vel -vel 0.0;...
              -vel -vel -vel -vel -vel -vel -vel 0.0;... 
              -acc -acc -acc -acc -acc -acc -acc 0.0;...
              -acc -acc -acc -acc -acc -acc -acc 0.0];

max_values = [2.0 2.0 2.0 2.0 2.0 2.0 2.0 2.0;... 
              vel vel vel vel vel vel vel 0.0;... 
              vel vel vel vel vel vel vel 0.0;...  
              acc acc acc acc acc acc acc 0.0;... 
              acc acc acc acc acc acc acc 0.0];

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 1;
    splineDiscretization = 20;
    maxIterationsSplineTCP = 50;
    visualizeTCPPath = 1;
    saveEMI = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%Variablen%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%ZRot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    offsetToBorder = 10; %Z.B 5
    minIter = 1000;%120
    maxiter = 2000;%2000
    span_to_Smooth = 0.005;%0.025 %As value from 0 to 1  !!!!MAX 0.1 SONST FEHLERANFÄLLIG!!!!!
    stepsize = 20;%10
    widhtStuetzp = 0; % Grade Zahl z.B 4 oder 0
    grenzSchwap_Y = 2.5;%STANDART 2.5!!!!


    optiZ_Rot_Param = [offsetToBorder,minIter,maxiter,span_to_Smooth,stepsize,widhtStuetzp,grenzSchwap_Y];
    optiZ_Rot_Param1 = optiZ_Rot_Param;
    
booleanManualPostProcessing = 0;

booleanTimeOpimizationTure = 0;
    maxIterations = 20;
    timeStepSize = 0.06; % nicht unter 0.05

booleanSloshingKompensationTrans = 0;
    numOfIterations = 30;

booleanSloshingKompensationRot = 0;

booleanVisualisation = 0;

%% ======== Optimierung: Beschl.-Profil TCP u. Erzeugung: Base-Points =====
if booleanFormTCP
    % Set up Boundarys für first Simulation
    jerkBoundaries = 0.5; % Für die Ruckänderung an den Mittelpunkten gilt +- dieser Wert als Max. bzw. Min.
    optimalSplineDiscretization = 80;

    % Initiale Werte für die Optimierung setzen
    init_ax_values = [timeSteps; VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   

    % Visualisieren der TCP-Bahn u. Speichern der Ergebnisse im EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, init_ax_values, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)
    for t =1:6
    % Zeitinervalle und Position der Kollisionspunkte optimieren
    [x_xy, optiResuls] = splineOptimization(optimalSplineDiscretization, maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values) 

    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xy, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)
    init_ax_values = x_xy;
    end
    % Profil in Z Richtung identifizieren
    VelocityBoundaryCondition_x =       [1.89324092806605	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	0.0]
    AccelerationBoundaryCondition_x =   [1.89324092806605	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	1.11626424791510	0.0]
    
    VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x]
    AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x]

    min_values = [x_xy(1,:); 
              -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 0.0; 
              -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 -4.5 0.0]

    max_values = [x_xy(1,:); 
                  4.5 4.5 4.5 4.5 4.5 4.5 4.5 0.0;  
                  4.5 4.5 4.5 4.5 4.5 4.5 4.5 0.0]

    % Initiale Werte für die Optimierung setzen
    init_ax_values = [x_xy(1,:); VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   

    [x_z, optiResuls] = splineOptimization_z(optimalSplineDiscretization, maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values)

    x_xyz(1,:) = x_xy(1,:);
    x_xyz(2,:) = x_xy(2,:);
    x_xyz(3,:) = x_xy(3,:);
    x_xyz(4,:) = x_z(2,:);
    x_xyz(5,:) = x_xy(4,:);
    x_xyz(6,:) = x_xy(5,:);
    x_xyz(7,:) = x_z(3,:);

    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xyz, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)

    % Generieren aller Bais-Points
    [x] = backwardTransformationRoboDK(Position_xyz, timeLine, splineDiscretization, x_xyz, axesPointConfigs,eulerZYX);
    
    save('SimResults.mat','x','-v7.3');
end

%% ======== Visualisierung u. Händische Nachbearbeitung ===================
if booleanManualPostProcessing
    [test] = ManualPostProcessing(splineDiscretization, visualizeTCPPath)
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
        [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations)
        
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