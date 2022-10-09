close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================
axesPointConfigs = transpose(deg2rad( ...
               [135.299231, -70.046429, 133.215319, -166.669123, 63.794111, 84.026271;
                97.076249, -61.042625, 114.372749, -171.202400, 53.654381, 84.759359;
                87.076249, -61.042625, 114.372749, -171.202400, 53.654381, 84.759359;
                77.076249, -61.042625, 114.372749, -171.202400, 53.654381, 84.759359;
                66.749562, -72.967077, 139.240207, -169.369934, 66.636337, 85.743286]));

tpts =                              [1.2 1.6 2.4 3.0];

VelocityBoundaryCondition_x =       [1 0.5 1  0.0]
AccelerationBoundaryCondition_x =   [-2 -1 -0  0.0]

VelocityBoundaryCondition_y =       [-0.3 0 1  0.0]
AccelerationBoundaryCondition_y =   [ 2 2 1  0.0]

VelocityBoundaryCondition_z =       [0 0 0  0.0]
AccelerationBoundaryCondition_z =   [0 0 0  0.0]

% Aufbau Value:
            % Zeitpunkte
            % VelocityBoundaryCondition_x
            % VelocityBoundaryCondition_y
            % VelocityBoundaryCondition_z
            % AccelerationBoundaryCondition_x
            % AccelerationBoundaryCondition_y
            % AccelerationBoundaryCondition_z
min_values = [0.2 0.2 0.2 1.0; 
              -3.2 -3.2 -3.2 0.0;
              -3.2 -3.2 -3.2 0.0; 
              -3.2 -3.2 -3.2 0.0; 
              -6.5 -6.5 -6.5 0.0; 
              -2.5 -2.5 -2.5 0.0; 
              -7.0 -7.0 -7.0 0.0]

max_values = [2.8 3.8 4.8 6.0; 
              3.2 3.2 3.2 0.0; 
              3.2 3.2 3.2 0.0; 
              3.2 3.2 3.2 0.0; 
              6.5 6.5 6.5 0.0; 
              2.5 2.5 2.5 0.0; 
              7.0 7.0 7.0 0.0]

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 1;
    splineDiscretization = 20;
    maxIterationsSplineTCP = 100;
    visualizeTCPPath = 1;
    saveEMI = 1;
    
booleanManualPostProcessing = 0;

booleanTimeOpimizationTure = 0;
    maxIterations = 60;
    timeStepSize = 0.06; % nicht unter 0.05

booleanSloshingKompensationTrans = 1;
    numOfIterations = 30;

booleanSloshingKompensationRot = 0;

booleanVisualisation = 0;

%% ======== Optimierung: Beschl.-Profil TCP u. Erzeugung: Base-Points =====
if booleanFormTCP
    % Set up Boundarys für first Simulation
    jerkBoundaries = 0.1 % Für die Ruckänderung an den Mittelpunkten gilt +- dieser Wert als Max. bzw. Min.

    VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x; VelocityBoundaryCondition_y; VelocityBoundaryCondition_z]
    AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x; AccelerationBoundaryCondition_y; AccelerationBoundaryCondition_z]

    % Initiale Werte für die Optimierung setzen
    init_ax_values = [tpts; 
                  VelocityBoundaryCondition_xyz_middle; 
                  AccelerationBoundaryCondition_xyz_middle];

    x = init_ax_values;
    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)


    % Zeitinervalle und Position der Kollisionspunkte optimieren
    [x, optiResuls] = splineOptimization(maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values) 

    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)

    % Generieren aller Bais-Points
    [x] = backwardTransformationRoboDK(Position_xyz, timeLine, splineDiscretization, startConfig, middleConfigs, goalConfig)
    
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
        [x,fval,eflag,output] = optimisationTrowelOrientation(numOfIterations, startConfig, goalConfig)
        
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