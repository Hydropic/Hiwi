function [minJerkPath] = pfadGeneratorSimple(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition)
% spline mit allen Punkten (Start, einsMitte, zweiMitte, Ende) erzeugen
% Die Werter vom Spline anschließend auf die gefürderten Zeitpunke übertragen
for time = 1:numSamples
    for axis = 1:length(middleOneConfig)
        if middleOneConfigUse
            if time <= middleOneConfigPosition
                difConfig = middleOneConfig - startConfig;
                if time == 1
                    teiler = 0;
                else
                    teiler = difConfig(1,axis)*(time/middleOneConfigPosition);
                end            
                minJerkPath(time,axis) = startConfig(1,axis) + teiler;
            else
                difConfig = goalConfig - middleOneConfig;
                if time == 1
                    teiler = 0;
                else
                    teiler = difConfig(1,axis)*((time-middleOneConfigPosition)/(numSamples-middleOneConfigPosition));
                end            
                minJerkPath(time,axis) = middleOneConfig(1,axis) + teiler;
            end         
        else
                difConfig = goalConfig - startConfig;
                if time == 1
                    teiler = 0;
                else
                    teiler = difConfig(1,axis)/(numSamples/time);
                end            
                minJerkPath(time,axis) = startConfig(1,axis) + teiler;
        end
    end
end

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

