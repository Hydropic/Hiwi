close all;
clear all;

%% ======== Setzen der Start-, End- und Kollisionspunkte ==================
startConfig = deg2rad([135.299231, -70.046429, 133.215319, -166.669123, 63.794111, 84.026271]);
goalConfig  = deg2rad([66.749562, -72.967077, 139.240207, -169.369934, 66.636337, 85.743286]);

middleOneConfigUse = true;
middleOneConfig  = deg2rad([97.076249, -61.042625, 114.372749, -171.202400, 53.654381, 84.759359]);
middleOneConfigPosition = 12;

middleTwoConfigUse = false;
middleTwoConfig  = deg2rad([20.02, -60.61, 180.26, 71.82, -81.59, -24.3]);
middleTwoConfigPosition = 15;

%% ======== Simulation konfigurieren ======================================
booleanFormTCP = 0;
    splineDiscretization = 20;
    maxIterationsSplineTCP = 10;
    visualizeTCPPath = 1;
    saveEMI = 1;
    
booleanManualPostProcessing = 0;

booleanTimeOpimizationTure = 0;
    maxIterations = 40;
    timeStepSize = 0.06; % nicht unter 0.05

booleanSloshingKompensationTrans = 1;
    numOfIterations = 30;

booleanSloshingKompensationRot = 0;

booleanVisualisation = 0;

%% ======== Optimierung: Beschl.-Profil TCP u. Erzeugung: Base-Points =====
if booleanFormTCP
    
    % Zeitinervalle und Position der Kollisionspunkte optimieren
    [x, optiResuls] = splineOptimization(maxIterationsSplineTCP, splineDiscretization, startConfig, middleOneConfig, goalConfig)  

    % Speichern der Ergebnisse in EMI Format
    [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x, splineDiscretization, startConfig, middleOneConfig, middleTwoConfig, goalConfig, middleOneConfigUse, middleTwoConfigUse, middleOneConfigPosition, middleTwoConfigPosition)

    % Generieren aller Bais-Points
    [x] = backwardTransformationRoboDK(Position_xyz, timeLine, splineDiscretization, startConfig, middleOneConfig, middleTwoConfig, goalConfig, middleOneConfigUse, middleTwoConfigUse, middleOneConfigPosition, middleTwoConfigPosition)
    
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
        
        optimized_translational_values_load(:,7) = x(:,3)

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