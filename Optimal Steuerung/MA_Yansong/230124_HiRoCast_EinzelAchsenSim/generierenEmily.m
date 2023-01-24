function [] = generierenEmily(optimized_translational_values)
splineDiscretization = 50;

    %% =============== Set of equal distances ================
    % TODO: Spline erzeugen
     sumTime(:,1) = optimized_translational_values(:,1);
    for j = 1:length(optimized_translational_values)       
        if j == 1
            sumTime(j,1) = 0;
        else
            sumTime(j,1) = sumTime(j-1,1) + sumTime(j,1)
        end
    end     

    optimized_translational_values_sameDistances = [];
    for k = 1:width(optimized_translational_values)        
        if k == 1
            optimized_translational_values_sameDistances(:, 1) = linspace(0, sumTime(end,1), splineDiscretization);

        else
            optimized_translational_values_sameDistances(:, k) = spline(sumTime(:,1), optimized_translational_values(:,k), optimized_translational_values_sameDistances(:, 1)); 
        end               
    end

    neueAchswinkel = optimized_translational_values_sameDistances;
    for x = 1:length(neueAchswinkel)
        [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(neueAchswinkel(x,2:7));
        optimized_translational_values_sameDistances(x,2) = pos(1);
        optimized_translational_values_sameDistances(x,3) = pos(2);
        optimized_translational_values_sameDistances(x,4) = pos(3);
        optimized_translational_values_sameDistances(x,5) = eulerZYX(1)-90;
        optimized_translational_values_sameDistances(x,6) = eulerZYX(2);
        optimized_translational_values_sameDistances(x,7) = eulerZYX(3)+90;
    end



    
    %% =============== Generate TCP Emily ================
    example = readcell('Emily1_blanko.txt');
    example(end+1,1) = {'[HEADER]'};
    example(end+1,1) = {'  GEAR_NOMINAL_VEL = 1.000000'};
    example(end+1,1) = {'  SAMPLING_MODE = CARTESIAN'};
    example(end+1,1) = {'  CRC = 4230818067'};
    example(end+1,1) = {'[RECORDS]'};

    for i = 1:length(optimized_translational_values_sameDistances)
% 
%         optimized_translational_values(i,5) = deg2rad(optimized_translational_values(i,5))
%         optimized_translational_values(i,6) = deg2rad(optimized_translational_values(i,6))
%         optimized_translational_values(i,7) = deg2rad(optimized_translational_values(i,7))

        if i > 1
            optimized_translational_values_sameDistances(i,1) = optimized_translational_values_sameDistances(i,1);            
        end
        optimized_translational_values_sameDistances(i,2) = (optimized_translational_values_sameDistances(i,2))*1000;
        optimized_translational_values_sameDistances(i,3) = (optimized_translational_values_sameDistances(i,3))*1000;
        optimized_translational_values_sameDistances(i,4) = (optimized_translational_values_sameDistances(i,4))*1000;
        optimized_translational_values_sameDistances(i,5) = (optimized_translational_values_sameDistances(i,5));
        optimized_translational_values_sameDistances(i,6) = (optimized_translational_values_sameDistances(i,6));
        optimized_translational_values_sameDistances(i,7) = (optimized_translational_values_sameDistances(i,7));

        optimized_translational_values_sameDistances(:) = optimized_translational_values_sameDistances(:);
        y = round(optimized_translational_values_sameDistances(:) * 100000)/100000;


        optimized_translational_values_sameDistances(i,:) = round(optimized_translational_values_sameDistances(i,:),4);
        txt = optimized_translational_values_sameDistances(i, :);

        txt = string(txt); 
        for x = 1:length(txt)
            txt(1,x) = string(sprintf('%.6f',txt(1,x)))
            if startsWith(txt(1,x),'-')                
            else
                txt(1,x) = '+' + txt(1,x)
            end          
        end
%         if i == 1
%             txtNull = txt;
%             txtNull(1,1) = "+0.000000" 
%             txtNull = strjoin(txtNull(:))
%             txtNull = '  ' + txtNull + ' +22 +50';
%             % txtNull = '  ' + txtNull;
%             example(end+1,1) = cellstr(txtNull(:));
%         end
        txt = strjoin(txt(:))
        % txt = '  ' + txt + ' +22 +50';
        txt = txt + ' +22 +50';
        example(end+1,1) = cellstr(txt(:));
    end
    example(end+1,1) = {'[END]'};
    
    mask = cellfun(@(x) any(isa(x,'missing')), example); % using isa instead of ismissing allows white space through
    example(mask) = {['']}
    
    writecell(example, 'Emily1_TCP.txt','Delimiter',' ')

    % Generate Start Position
    startAxes = [optimized_translational_values_sameDistances(1,2), optimized_translational_values_sameDistances(1,3), optimized_translational_values_sameDistances(1,4), optimized_translational_values_sameDistances(1,5) ,optimized_translational_values_sameDistances(1,6), optimized_translational_values_sameDistances(1,7)];
    [pos2, eulerZYX2,eulerXYZ2, y_direction2] = vorwaertskinematik(startAxes);
    startAxes(2,1) = pos2(1,1);
    startAxes(2,2) = pos2(2,1);
    startAxes(2,3) = pos2(3,1);
    startAxes(2,4) = eulerZYX2(1,1);
    startAxes(2,5) = eulerZYX2(1,2);
    startAxes(2,6) = eulerZYX2(1,3);

    Startvalues(1,1) = startAxes(1,1);
    Startvalues(2,1) = startAxes(1,2);
    Startvalues(3,1) = startAxes(1,3);
    Startvalues(4,1) = startAxes(1,4);
    Startvalues(5,1) = startAxes(1,5);
    Startvalues(6,1) = startAxes(1,6);

    Startvalues(1,2) = pos2(1,1);
    Startvalues(2,2) = pos2(2,1);
    Startvalues(3,2) = pos2(3,1);
    Startvalues(4,2) = eulerZYX2(1,1);
    Startvalues(5,2) = eulerZYX2(1,2);
    Startvalues(6,2) = eulerZYX2(1,3);

    fid=fopen('Emily1_TCP_start.txt','w');
    b=fprintf(fid,'%.6f\n',Startvalues)
    fclose(fid)

%% ============== Generate Axis Emily ===============================================================================================
    example = readcell('Emily1_blanko.txt');
    example(end+1,1) = {'[HEADER]'};
    example(end+1,1) = {'  GEAR_NOMINAL_VEL = 1.000000'};
    example(end+1,1) = {'  CRC = 4230818067'};
    example(end+1,1) = {'[RECORDS]'};


    sumTime(:,1) = optimized_translational_values(:,1);
    for j = 1:length(optimized_translational_values)       
        if j == 1
            sumTime(j,1) = 0;
        else
            sumTime(j,1) = sumTime(j-1,1) + sumTime(j,1)
        end
    end     

    optimized_translational_values_sameDistances_axis = [];
    for k = 1:width(optimized_translational_values)        
        if k == 1
            optimized_translational_values_sameDistances_axis(:, 1) = linspace(0, sumTime(end,1), splineDiscretization);

        else
            optimized_translational_values_sameDistances_axis(:, k) = spline(sumTime(:,1), optimized_translational_values(:,k), optimized_translational_values_sameDistances_axis(:, 1)); 
        end               
    end

    for i = 1:length(optimized_translational_values_sameDistances_axis)
% 
%         optimized_translational_values_sameDistances_axis(i,5) = deg2rad(optimized_translational_values_sameDistances_axis(i,5))
%         optimized_translational_values_sameDistances_axis(i,6) = deg2rad(optimized_translational_values_sameDistances_axis(i,6))
%         optimized_translational_values_sameDistances_axis(i,7) = deg2rad(optimized_translational_values_sameDistances_axis(i,7))

        if i >1
            optimized_translational_values_sameDistances_axis(i,1) = optimized_translational_values_sameDistances_axis(i,1)            
        end
        optimized_translational_values_sameDistances_axis(i,2) = rad2deg(optimized_translational_values_sameDistances_axis(i,2));
        optimized_translational_values_sameDistances_axis(i,3) = rad2deg(optimized_translational_values_sameDistances_axis(i,3));
        optimized_translational_values_sameDistances_axis(i,4) = rad2deg(optimized_translational_values_sameDistances_axis(i,4));
        optimized_translational_values_sameDistances_axis(i,5) = rad2deg(optimized_translational_values_sameDistances_axis(i,5));
        optimized_translational_values_sameDistances_axis(i,6) = rad2deg(optimized_translational_values_sameDistances_axis(i,6));
        optimized_translational_values_sameDistances_axis(i,7) = rad2deg(optimized_translational_values_sameDistances_axis(i,7));

        optimized_translational_values_sameDistances_axis(:) = optimized_translational_values_sameDistances_axis(:);
        y = round(optimized_translational_values_sameDistances_axis(:) * 100000)/100000;


        optimized_translational_values_sameDistances_axis(i,:) = round(optimized_translational_values_sameDistances_axis(i,:),4);
        txt = optimized_translational_values_sameDistances_axis(i, :);

        txt = string(txt); 
        asdasdasda = length(txt);
        for x = 1:length(txt)
            txt(1,x) = string(sprintf('%.6f',txt(1,x)))
            if startsWith(txt(1,x),'-')                
            else
                txt(1,x) = '+' + txt(1,x)
            end          
        end

        txt = strjoin(txt(:))
        txt = '  ' + txt;
        %txt = '  ' + txt + ' +22 +50';
        example(end+1,1) = cellstr(txt(:));
    end
    example(end+1,1) = {'[END]'};
    
    mask = cellfun(@(x) any(isa(x,'missing')), example); % using isa instead of ismissing allows white space through
    example(mask) = {['']}
    
    writecell(example, 'Emily1_Axis.txt','Delimiter',' ')

    % Generate Start Position
    startAxes = [optimized_translational_values_sameDistances_axis(1,2), optimized_translational_values_sameDistances_axis(1,3), optimized_translational_values_sameDistances_axis(1,4), optimized_translational_values_sameDistances_axis(1,5) ,optimized_translational_values_sameDistances_axis(1,6), optimized_translational_values_sameDistances_axis(1,7)];
    [pos2, eulerZYX2,eulerXYZ2, y_direction2] = vorwaertskinematik(startAxes);
    startAxes(2,1) = pos2(1,1);
    startAxes(2,2) = pos2(2,1);
    startAxes(2,3) = pos2(3,1);
    startAxes(2,4) = eulerZYX2(1,1);
    startAxes(2,5) = eulerZYX2(1,2);
    startAxes(2,6) = eulerZYX2(1,3);

    Startvalues(1,1) = startAxes(1,1);
    Startvalues(2,1) = startAxes(1,2);
    Startvalues(3,1) = startAxes(1,3);
    Startvalues(4,1) = startAxes(1,4);
    Startvalues(5,1) = startAxes(1,5);
    Startvalues(6,1) = startAxes(1,6);

    Startvalues(1,2) = pos2(1,1);
    Startvalues(2,2) = pos2(2,1);
    Startvalues(3,2) = pos2(3,1);
    Startvalues(4,2) = eulerZYX2(1,1);
    Startvalues(5,2) = eulerZYX2(1,2);
    Startvalues(6,2) = eulerZYX2(1,3);

    fid=fopen('Emily1_Axis_start.txt','w');
    b=fprintf(fid,'%.6f\n',Startvalues)
    fclose(fid)

end
