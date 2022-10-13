function [minJerkPath] = pfadGeneratorSimple(splineDiscretization, numSamples, axesPointConfigs, timeLine, x_xyz)
    % spline mit allen Punkten (Start, einsMitte, zweiMitte, Ende) erzeugen
    % Die Werter vom Spline anschließend auf die gefürderten Zeitpunke übertragen
    [breite, hoehe] = size(axesPointConfigs)
    for k = 1:breite
        % spline aus den Punkten erzeugen
        timeSteps = [];
        for t = 0:length(x_xyz)
            if t == 0
                timeSteps(t+1) = 0
            else
                timeSteps(t+1) = x_xyz(1, t) + timeSteps(1, t)
            end
        end
        
        x = timeSteps;
        y = axesPointConfigs(k,:);
        
        interval =  timeLine(end)/19
        % xx = 0:interval:timeLine(end);
        xx = 0:interval:timeLine(end);
        yy = spline(x,y,xx);
%         figure(yy)
%         minJerkPath(k,:) = yy(1,:)
    end
end
% [breite, hoehe] = size(axesPointConfigs)
% for time = 1:numSamples
%     for axis = 1:length(breite)
%         if middleOneConfigUse
%             if time <= middleOneConfigPosition
%                 difConfig = middleOneConfig - startConfig;
%                 if time == 1
%                     teiler = 0;
%                 else
%                     teiler = difConfig(1,axis)*(time/middleOneConfigPosition);
%                 end            
%                 minJerkPath(time,axis) = startConfig(1,axis) + teiler;
%             else
%                 difConfig = goalConfig - middleOneConfig;
%                 if time == 1
%                     teiler = 0;
%                 else
%                     teiler = difConfig(1,axis)*((time-middleOneConfigPosition)/(numSamples-middleOneConfigPosition));
%                 end            
%                 minJerkPath(time,axis) = middleOneConfig(1,axis) + teiler;
%             end         
%         else
%                 difConfig = goalConfig - startConfig;
%                 if time == 1
%                     teiler = 0;
%                 else
%                     teiler = difConfig(1,axis)/(numSamples/time);
%                 end            
%                 minJerkPath(time,axis) = startConfig(1,axis) + teiler;
%         end
%     end
% end