function [x, optiResuls] = timeOptimization(timeStepSize, maxIterations, rseriesSimTrue, startConfig, goalConfig, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition)
global axis;
global seriesSim;
global fixedTime;
close all;      
clearvars -except timeStepSize maxIterations axis seriesSim rseriesSimTrue startConfig goalConfig middleOneConfigUse middleOneConfig middleOneConfigPosition middleTwoConfigUse middleTwoConfig middleTwoConfigPosition fixedTime

        if rseriesSimTrue
           axis = seriesSim(axis);
        end
        if fixedTime
            example = matfile('SimResults.mat');
            optimized_translational_values = example.x;
            minJerkPath = optimized_translational_values(:, 2:7)
            timeSum = sum(optimized_translational_values(:, 1));
            % timeStepSize = timeSum/length(optimized_translational_values);   
            % show_spline(optimized_translational_values, 'Bad Output');
        else            
%             example = matfile('SimResults.mat');
%             optimized_translational_values = example.x;
%             minJerkPath = optimized_translational_values(:, 2:7)
%             timeSum = sum(optimized_translational_values(:, 1));

            numSamples = 20;
            for t = 1:numSamples-1
                timesteps(t, 1) = timeStepSize
            end
            % minJerkPath = pfadGenerator(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition);
            minJerkPath = pfadGeneratorProfile(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition);
            x(1:20,1) = timesteps(1,1)
            x(:,2:7) = minJerkPath
            show_spline(x, 'y (um Achse 6), x (um Achse 5)'); % SHOW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        end 
        % plot(minJerkPath(:,:))
        % input der achsstellungen
        init_ax_values = [ones(size(minJerkPath,1),1) minJerkPath];


%         % händisches strecken oder minimieren der Zeit
%         for t = 1:20
%         optimized_translational_values(t, 1) = optimized_translational_values(t, 1) - 0.03*(t/20)^3
%         end

        if fixedTime
            init_ax_values(:,1) = optimized_translational_values(:, 1);
        else 
            init_ax_values(:,1) = optimized_translational_values(:, 1); %Anfängliche Beschleunigung = 0,3m/s???
            % init_ax_values(:,1) = timeStepSize; %Anfängliche Beschleunigung = 0,3m/s???
        end

        
        % Defineig options für das Optimalsteuerungsprob.
        opts = optimoptions(@fmincon, ...
            'Algorithm','interior-point', ...
            "MaxFunctionEvaluations",50000,...
            "MaxIterations",maxIterations, ...
            "StepTolerance",1e-17, ...
            "OptimalityTolerance",1e-17, ...
            "EnableFeasibilityMode",true, ...
            "DiffMinChange", 0.01, ... 
            "DiffMaxChange", 2, ...
            "SubproblemAlgorithm",'factorization', ...
            "PlotFcn",["optimplotfunccount","optimplotfvalconstr","optimplotconstrviolation","optimplotstepsize","optimplotfirstorderopt"], ...
            "Display",'iter','ConstraintTolerance',0.00018);
        
        % Definig ub lb and options
        max_jointangle = deg2rad([185,14,144,350,120,350]);
        min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);

        if false % fixedTime
            min_values = init_ax_values;
            max_values = init_ax_values;
        else
            min_values = repmat(cat(2,[0.05],min_jointangle),[size(init_ax_values,1),1]); 
            max_values = repmat(cat(2,[0.13],max_jointangle),[size(init_ax_values,1),1]); % 0.2
        end
        
        [eins, zwei] = size(max_values);
        % Fix all not to optimize Axis 
        for a = 1:width(minJerkPath)
            if a == axis
                min_values(:, a+1) = min_jointangle(a);
                max_values(:, a+1) = max_jointangle(a);
            else
                min_values(:, a+1) = minJerkPath(:, a);
                max_values(:, a+1) = minJerkPath(:, a);
            end
        end

%         for i = 1:eins
%             for a = 1:width(minJerkPath)
%                 if a == axis
%                     min_values(i, a+1) = min_jointangle(a);
%                     max_values(i, a+1) = max_jointangle(a);
%                 else
%                     min_values(i, a+1) = minJerkPath(i, a);
%                     max_values(i, a+1) = minJerkPath(i, a);
%                 end
%             end
%         end
        
        
        % Fix Start and End Position
        for i = 1:zwei-1
            min_values(1, i+1) = minJerkPath(1, i);
            max_values(1, i+1) = minJerkPath(1, i);
            min_values(end, i+1) = minJerkPath(end, i);
            max_values(end, i+1) = minJerkPath(end, i);
        end
        
        % Fix middleOne Position
        if middleOneConfigUse
            for i = 1:zwei-1
                if fixedTime
                        min_values(middleOneConfigPosition, axis+1) = middleOneConfig(axis) - 0.05;
                        max_values(middleOneConfigPosition, axis+1) = middleOneConfig(axis) + 0.05;
                else
                        min_values(middleOneConfigPosition, i+1) = middleOneConfig(i) - 0.06;
                        max_values(middleOneConfigPosition, i+1) = middleOneConfig(i) + 0.06;
                end
            end
        end
        
        % Fix middleTwo Position
        if middleTwoConfigUse
            for i = 1:zwei-1
                min_values(middleTwoConfigPosition, i+1) = middleTwoConfig(i) %- 0.05;
                max_values(middleTwoConfigPosition, i+1) = middleTwoConfig(i) %- 0.05;
            end
        end
        
        % plot(minJerkPath)
        
        % [eins, zwei] = size(init_ax_values);
        % for i = 1:eins-1
        %     for a = 1:zwei
        %         init_ax_values(i,a) = 0.1;
        %     end
        % end
        optimization_values = init_ax_values;
        problem = createOptimProblem('fmincon',...
            'x0',init_ax_values, ...
            'objective',@optimization_task,...
            'nonlcon', @(optimization_values)constraintFcnValidation(optimization_values,init_ax_values,"translatorisch", axis), ...
            'lb',min_values,...
            'ub',max_values, ...
            'options',opts);
            
        [x,fval,eflag,output] = fmincon(problem);
        
        optiResuls = [fval, output.constrviolation]
    
        %% =========Winkelausschlag aus Pendelmodell=============================== 
        % [position_TCP, angleTCP, optimization_values, optimized_translational_values_clear] = prepairValuse_5_6(x, 1);
        
        % % Pendelsimulation aufrufen
        % [path_angular_deflection_old] = pendelSimulation(position_TCP); % path_angular_deflection = [Zeit, rotation um y, Rotation um x]
        % 
        % [timeSmoothSpline, smoothSpline_y, smoothSpline_x] = drawSpline(path_angular_deflection_old);
        %     for w = 1:length(x)
        %         [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(x(w,2:7));
        %         x(w,6) = eulerZYX(2);
        %         x(w,7) = eulerZYX(3);
        %     end
        %     show_spline(x, 'y (um Achse 6), x (um Achse 5)');
    end


    function objective = optimization_task(optimization_values)
        timeintervals = optimization_values(1:size(optimization_values,1)-1,1);
        base_points = optimization_values(:,2:size(optimization_values,2));
        objective = sum(timeintervals); 

        global axis;
        [achsstellung,vilocity,acceleration,jerk,time,splinePunkt] = splineOptimal(base_points(:,axis),timeintervals,false);    
        pos = jerk>0;
        changes = xor(pos(1:end-1),pos(2:end));
        num = sum(changes)   
        objective = objective % + 0.0000000021 * trapz((acceleration.^2)); %TODO 
        % objective = objective + 0.0000000006 * trapz((acceleration.^2)); %TODO 
    end