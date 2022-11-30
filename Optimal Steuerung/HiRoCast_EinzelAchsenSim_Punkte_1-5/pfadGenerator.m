function [minJerkPath] = pfadGenerator(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition)
        [lTime, hTime] = size(timesteps)
        virtuelPoint = 0;
        if middleOneConfigUse == 0 && middleTwoConfigUse == 0
            middleOneConfigUse = 1;            
            middleOneConfig = startConfig + (goalConfig - startConfig)/2;
            virtuelPoint = 1;
        end

        for t = 1:lTime
            if t == 1 
                sumTimeStep(1, 1) = timesteps(1, t)
            else
                sumTimeStep(t, 1) = sumTimeStep(t-1, 1) + timesteps(t, 1)
            end
        end
        A = [1,1; 2,2; 3,3]; 
        b = [0]; 
        k = 0; %row position, can be 0,1,2 or 3 in this case
        sumTimeStep = [b; sumTimeStep(k+1:end,:)]
    for axis = 1:length(middleOneConfig)
        
        if middleOneConfigUse == 1 & middleTwoConfigUse == 1
            winkel_x = [startConfig(1, axis), middleOneConfig(1, axis), middleTwoConfig(1, axis), goalConfig(1, axis)] 
            timestepsSpline = [1, middleOneConfigPosition, middleTwoConfigPosition, length(timesteps)];
        elseif middleOneConfigUse == 1 & middleTwoConfigUse == 0
            winkel_x = [startConfig(1, axis), middleOneConfig(1, axis), goalConfig(1, axis)] 
            timestepsSpline = [1, middleOneConfigPosition, length(timesteps)];
        elseif middleOneConfigUse == 0 & middleTwoConfigUse == 1
            winkel_x = [startConfig(1, axis), middleTwoConfig(1, axis), goalConfig(1, axis)] 
            timestepsSpline = [1, middleTwoConfigPosition, length(timesteps)];
        else
            winkel_x = [startConfig(1, axis), goalConfig(1, axis)] 
            timestepsSpline = [1, length(timesteps)];
        end
        
        winkel_x_6 = [winkel_x', winkel_x', winkel_x', winkel_x', winkel_x', winkel_x']; 
        for p = 1:length(timestepsSpline)
            % für jeden Timestep die Differenz zum vorherigen berechnen
            if p == 1
            elseif p == 2
                timestepsSplineUpdate(p-1) = sum(timesteps(timestepsSpline(p-1):timestepsSpline(p),1))
            else
                timestepsSplineUpdate(p-1) = sum(timesteps(timestepsSpline(p-1)+1:timestepsSpline(p),1))
            end
        end
        [achsstellung_x,vilocity_x,acceleration_x,yerk_x,timeAbs_x,splinePunkt_x] = splineOptimal(winkel_x_6,timestepsSplineUpdate,false);

    
        for l = 1:numSamples
%             if l == 1
%                 minJerkPath(l, axis) = achsstellung_x(1)
%             else
                [ d, ix ] = min(abs(timeAbs_x-sumTimeStep(l, 1)));            
                minJerkPath(l, axis) = achsstellung_x(ix)
                test = 2;
%             end
    
        end
    

%         figure;
%         plot(timeAbs_x, achsstellung_x);
%         figure;
%         plot(sumTimeStep, minJerkPath')
    end

    if virtuelPoint == 1
        middleOneConfigUse = 0;            
        virtuelPoint = 0;
    end

end

% spline mit allen Punkten (Start, einsMitte, zweiMitte, Ende) erzeugen
% Die Werter vom Spline anschließend auf die gefürderten Zeitpunke übertragen

% for time = 1:numSamples
%         for axis = 1:length(middleOneConfig)
%             isNoMiddleCinfig = true;
%             if time == middleOneConfigPosition
%                 if middleOneConfigUse
%                     minJerkPath(time,axis) = middleOneConfig(1,axis)
%                     isNoMiddleCinfig = false;
%                 end
%             elseif time == middleTwoConfigPosition
%                 if middleTwoConfigUse
%                     minJerkPath(time,axis) = middleTwoConfig(1,axis)
%                     isNoMiddleCinfig = false;
%                 end
%             else
%             end
%             if isNoMiddleCinfig
%                 difConfig = goalConfig - startConfig;
%                 if time == 1
%                     teiler = 0;
%                 else
%                     teiler = difConfig(1,axis)/(numSamples/time);
%                 end            
%                 minJerkPath(time,axis) = startConfig(1,axis) + teiler;
%             end
%         end
%     end
% end

