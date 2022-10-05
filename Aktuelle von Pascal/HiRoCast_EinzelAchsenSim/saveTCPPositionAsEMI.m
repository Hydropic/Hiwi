function [Position_xyz, timeLine] = saveTCPPositionAsEMI(visualizeTCPPath, saveEMI, x, splineDiscretization, startConfig, middleOneConfig, middleTwoConfig, goalConfig, middleOneConfigUse, middleTwoConfigUse, middleOneConfigPosition, middleTwoConfigPosition)  
    optimization_values = x;

    achsstellungen(1,:) = startConfig
    achsstellungen(2,:) = middleOneConfig
    achsstellungen(3,:) = goalConfig
    wayPoints = [];

    for p = 1:height(achsstellungen)
        [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(achsstellungen(p,:))
        wayPoints(:,p) = tcppunkt(:,1)
    end

    [Position_xyz, timeLine] = generateTCPPath(optimization_values, wayPoints, splineDiscretization, visualizeTCPPath)

    if saveEMI
        optimized_translational_values_sameDistances(:, 1) = transpose(timeLine);
        optimized_translational_values_sameDistances(:, 2:4) = 1000*Position_xyz;
        optimized_translational_values_sameDistances(:, 5) = -90;
        optimized_translational_values_sameDistances(:, 6) = 0;
        optimized_translational_values_sameDistances(:, 7) = +90;
    
        example = readcell('Emily1_blanko.txt');
        example(end+1,1) = {'[HEADER]'};
        example(end+1,1) = {'  GEAR_NOMINAL_VEL = 1.000000'};
        example(end+1,1) = {'  SAMPLING_MODE = CARTESIAN'};
        example(end+1,1) = {'  CRC = 4230818067'};
        example(end+1,1) = {'[RECORDS]'};
    
        for i = 1:length(optimized_translational_values_sameDistances)
    
            txt = optimized_translational_values_sameDistances(i, :);
    
            txt = string(txt); 
            for x = 1:length(txt)
                txt(1,x) = string(sprintf('%.6f',txt(1,x)))
                if startsWith(txt(1,x),'-')                
                else
                    txt(1,x) = '+' + txt(1,x)
                end          
            end
    
            txt = strjoin(txt(:))
            % txt = '  ' + txt + ' +22 +50';
            txt = txt + ' +22 +50';
            example(end+1,1) = cellstr(txt(:));
        end
        example(end+1,1) = {'[END]'};
        
        mask = cellfun(@(x) any(isa(x,'missing')), example); % using isa instead of ismissing allows white space through
        example(mask) = {['']}
        
        writecell(example, 'Emily1_TCP_formTransXY.txt','Delimiter',' ')
    end
end
