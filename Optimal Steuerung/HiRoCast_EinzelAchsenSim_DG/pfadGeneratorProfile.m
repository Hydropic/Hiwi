function [minJerkPath] = pfadGeneratorProfile(timesteps, startConfig, goalConfig, numSamples, middleOneConfigUse, middleOneConfig, middleOneConfigPosition, middleTwoConfigUse, middleTwoConfig, middleTwoConfigPosition)

[splinePath] = show6DoF(numSamples)
splinePathValuePlotGlobal = [];

        for axis = 1:length(middleOneConfig)
%             if startConfig(1, axis) > goalConfig(1, axis) && startConfig(1, axis) < middleOneConfig(1, axis)
%                 % Profil + -Profil
%                 test = 0;
%             elseif startConfig(1, axis) > goalConfig(1, axis) && goalConfig(1, axis) > middleOneConfig(1, axis)
%                 % -Profil + Profil
%                 test = 0;
            if startConfig(1, axis) > goalConfig(1, axis)
                % -Profil
                for p = 1:length(splinePath)
                    splinePathValue(p) = startConfig(axis) - (startConfig(axis)-goalConfig(axis))*splinePath(p, 2)
                end
                % plot(splinePathValue)
%             elseif startConfig(1, axis) < goalConfig(1, axis) && goalConfig(1, axis) < middleOneConfig(1, axis)
%                 % Profil + -Profil
%                 test = 0;
%             elseif startConfig(1, axis) < goalConfig(1, axis) && startConfig(1, axis) > middleOneConfig(1, axis)
%                 % -Profil + Profil
%                 test = 0;
            elseif startConfig(1, axis) < goalConfig(1, axis)
                % Profil
                for p = 1:length(splinePath)
                    splinePathValue(p) = startConfig(axis) + (goalConfig(axis)-startConfig(axis))*splinePath(p, 2)
                end
                % plot(splinePathValue)
            end
            splinePathValueInvert = splinePathValue'
            % Mittleren Punkt einfügen
%             if axis == 1
%                 factor = -0.05
%             elseif axis == 2
%                 factor = 0.075
%             elseif axis == 3
%                 factor = 0.067
%             elseif axis == 4
%                 factor = 0.00
%             elseif axis == 5
%                 factor = 0.00
%             elseif axis == 6
%                 factor = 0.00
%             end 

            if axis == 1
                factor = -0.0345
            elseif axis == 2
                factor = 0.026
            elseif axis == 3
                factor = 0.0205
            elseif axis == 4
                factor = 0.00
            elseif axis == 5
                factor = 0.00
            elseif axis == 6
                factor = 0.00
            end 
            
            splinePathValuePlot(1:length(splinePathValueInvert),1) = timesteps(1,1);
            splinePathValuePlot(:,2) = splinePathValueInvert(:,1)
            splinePathValuePlot(:,3) = splinePathValueInvert(:,1)
            splinePathValuePlot(:,4) = splinePathValueInvert(:,1)
            splinePathValuePlot(:,5) = splinePathValueInvert(:,1)
            splinePathValuePlot(:,6) = splinePathValueInvert(:,1)
            splinePathValuePlot(:,7) = splinePathValueInvert(:,1)
            
            for as = 1:length(splinePathValuePlot)
                pos = ((length(splinePathValueInvert)-(as-1))/length(splinePathValueInvert))
                %sddsd = as*splinePathValue(as,2)*factor
                splinePathValuePlot(as,1) = splinePathValuePlot(as,1) + (pos)^2*factor % - sddsd
            end
        
            for as = length(splinePathValuePlot):-1:1
                pos = as/length(splinePathValueInvert)
                %sddsd = as*splinePathValue(as,2)*factor
                splinePathValuePlot(as,1) = splinePathValuePlot(as,1) - (pos)^2*factor % - sddsd
            end

            for d = 1:length(splinePathValuePlot)
                if d == 1
                    splinePathValuePlot(d,1) = splinePathValuePlot(d,1);
                else
                    splinePathValuePlot(d,1) = splinePathValuePlot(d,1)+splinePathValuePlot(d-1,1)
                end
                
            end 

            
            

            splineDiscretization = 200;
            
            lin_xx_x = linspace(0, splinePathValuePlot(end, 1), splineDiscretization);
            lin_yy_x = spline(splinePathValuePlot(:, 1), splinePathValuePlot(:, 2), lin_xx_x);
            lin_xx_x = lin_xx_x(1,splineDiscretization/numSamples:splineDiscretization/numSamples:end);
            lin_yy_x = lin_yy_x(1,splineDiscretization/numSamples:splineDiscretization/numSamples:end);

            lin_xx_x_time = []
            for k = 1:length(splinePathValuePlot)
                if k ==1
                    lin_xx_x_time(k,1) = timesteps(1,1);
                else
                    lin_xx_x_time(k,1) = lin_xx_x_time(k-1,1) + timesteps(1,1);
                end
            end
%             figure
%             plot(lin_xx_x,lin_yy_x)
%             hold on; 
%             scatter(lin_xx_x(middleOneConfigPosition),middleOneConfig(axis))

            splinePathValuePlotGlobal(1:numSamples,1) = timesteps(1,1)
            splinePathValuePlotGlobal(:,axis+1) = lin_yy_x'         
            
            minJerkPath(:,axis) = splinePathValue(:)
        end
        % show_spline(splinePathValuePlotGlobal, 'y (um Achse 6), x (um Achse 5)');
        minJerkPath = splinePathValuePlotGlobal(:,2:7)
end

