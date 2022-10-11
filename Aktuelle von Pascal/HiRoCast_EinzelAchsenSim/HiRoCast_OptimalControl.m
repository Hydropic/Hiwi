close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================
axesPointConfigs = transpose(deg2rad( ...
               [172.878447, -47.894415, 90.739178, -131.159795, 54.639091, 56.495088;
                149.706722, -69.944576, 109.414811, -157.564321, 41.697767, 72.865636;
                120.805660, -58.687349, 115.013096, -196.905741, 57.483511, 99.278817;
                122.819557, -51.233913, 98.175757, -196.458086, 48.135560, 101.153011]));

tpts =                              [0.579449308069551	1.043651613947915	1.543651613947915];

timeSteps = [];
for t = 1:length(tpts)
    if t == 1
        timeSteps(t) = tpts(1, 1)
    else
        timeSteps(t) = tpts(1, t) - tpts(1, t-1)
    end
end

VelocityBoundaryCondition_x =       [1.89324092806605	1.11626424791510	0.0]
AccelerationBoundaryCondition_x =   [1.32791857795363	-3.31940823068824	0.0]


VelocityBoundaryCondition_y =       [-1.95367961344194	-1.18494663299643	0.0]
AccelerationBoundaryCondition_y =   [-2.90067458258856	4.48604587496325	0.0]

VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x; VelocityBoundaryCondition_y]
AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x; AccelerationBoundaryCondition_y]

% Aufbau Value:
            % Zeitpunkte
            % VelocityBoundaryCondition_x
            % VelocityBoundaryCondition_y
            % VelocityBoundaryCondition_z
            % AccelerationBoundaryCondition_x
            % AccelerationBoundaryCondition_y
            % AccelerationBoundaryCondition_z
min_values = [0.1 0.1 0.1; 
              -6.5 -6.5 0.0;
              -6.5 -6.5 0.0; 
              -6.5 -6.5 0.0; 
              -6.5 -6.5 0.0]

max_values = [1.5 1.5 1.5; 
              6.5 6.5 0.0; 
              6.5 6.5 0.0;  
              6.5 6.5 0.0; 
              6.5 6.5 0.0]

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 1;
    splineDiscretization = 20;
    maxIterationsSplineTCP = 30;
    visualizeTCPPath = 1;
    saveEMI = 0;
    
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
    jerkBoundaries = 0.7 % Für die Ruckänderung an den Mittelpunkten gilt +- dieser Wert als Max. bzw. Min.

    % Initiale Werte für die Optimierung setzen
    init_ax_values = [timeSteps; VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   

    % Visualisieren der TCP-Bahn u. Speichern der Ergebnisse im EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, init_ax_values, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)

    % Zeitinervalle und Position der Kollisionspunkte optimieren
    [x_xy, optiResuls] = splineOptimization(maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values) 

    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x_xy, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries)

    % Profil in Z Richtung identifizieren

    VelocityBoundaryCondition_x =       [1.89324092806605	1.11626424791510	0.0]
    AccelerationBoundaryCondition_x =   [1.32791857795363	-3.31940823068824	0.0]
    
    VelocityBoundaryCondition_xyz_middle = [VelocityBoundaryCondition_x]
    AccelerationBoundaryCondition_xyz_middle = [AccelerationBoundaryCondition_x]

    min_values = [x_xy(1,1) x_xy(1,2) x_xy(1,3); 
              -6.5 -6.5 0.0; 
              -6.5 -6.5 0.0]

    max_values = [x_xy(1,1) x_xy(1,2) x_xy(1,3); 
                  6.5 6.5 0.0;  
                  6.5 6.5 0.0]

    % Initiale Werte für die Optimierung setzen
    init_ax_values = [x_xy(1,:); VelocityBoundaryCondition_xyz_middle; AccelerationBoundaryCondition_xyz_middle];   

    [x_z, optiResuls] = splineOptimization_z(maxIterationsSplineTCP, splineDiscretization, axesPointConfigs, min_values, max_values, jerkBoundaries, init_ax_values)

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