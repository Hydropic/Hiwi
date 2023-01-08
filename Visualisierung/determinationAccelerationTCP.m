function [orientation_xy] = determinationAccelerationTCP(optimized_translational_values)
    position_TCP = [];

    EMIFile = "input/InputSOBGB_opti_DG_Emily_Axis.txt_Zeit_14.2.txt";

        alleXte = 150; 
        punkteVersetzen = [1 1 0.0;];

    lineOfEmi = regexp(fileread(EMIFile),'\n','split');
    startLine = find(contains(lineOfEmi,'[RECORDS]'));
    endLine = find(contains(lineOfEmi,'[END]'));
    
    endLine = endLine - 2;
    data = dlmread(EMIFile,'',[startLine 0 endLine 6]);
    
    
    %Put each axis in its own array
    timeData = data(:,1);
    first_Axis = data(:,2);
    second_Axis = data(:,3);
    third_Axis = data(:,4);
    fourth_Axis = data(:,5);
    fifth_Axis = data(:,6);
    sixth_Axis = data(:,7);

    timeInterval = [];

    for sss = 1:length(timeData)
        if sss == 1
            timeInterval(sss,1) = 0;
        else
            timeInterval(sss,1) = timeData(sss) - timeData(sss-1);
        end
        
    end

    optimized_translational_values(:,1) = timeInterval(:,1);
    optimized_translational_values(:,2) = deg2rad(data(:,2));
    optimized_translational_values(:,3) = deg2rad(data(:,3));
    optimized_translational_values(:,4) = deg2rad(data(:,4));
    optimized_translational_values(:,5) = deg2rad(data(:,5));
    optimized_translational_values(:,6) = deg2rad(data(:,6));
    optimized_translational_values(:,7) = deg2rad(data(:,7));



    summeZeitIntervalle = sum(optimized_translational_values(:,1));

    optimized_translational_values(1,1) = optimized_translational_values(2,1);
    
    % Transformation der Achsen in TCP Koordinaten
    for e = 1:length(optimized_translational_values)
        [pos2, eulerZYX2,~, ~] = vorwaertskinematik(optimized_translational_values(e, 2:7));          
        eulZYX(e,1:3) = transpose(eulerZYX2);
        position_TCP(end+1,1:3) = transpose(pos2(1:3,1));
    end  
   
    [euler_XY,~,~,~,time_Spline,~] = splineOptimal(eulZYX(:,1),optimized_translational_values(1:end-1,1),false);   
    
    for i = 1:3
        [~,~,acceleration,~,~,~] = splineOptimal(position_TCP(:,i),optimized_translational_values(1:end-1,1),false);           
        acceleration_xy(:,i) = acceleration;
    end
  
    %Rotation um Z
    for i = 1:size(euler_XY,2)
        acceleration_xy_TCP(i,1:3) = transpose(RotationDegUmZ(-euler_XY(i))*transpose(acceleration_xy(i,1:3)));          
    end
   
    %Umorienterung um schwappen durch beschleunigung zu Kompensieren
    for t = 1:length(acceleration_xy_TCP)
       orientation_xy(t,1) = time_Spline(1,t);
       orientation_xy(t,2) = atand(acceleration_xy_TCP(t,1)/(9.81+acceleration_xy_TCP(t,3))); % TODO: für 9.81 die Z Komponente mit rein bringen
       orientation_xy(t,3) = atand(acceleration_xy_TCP(t,2)/(9.81+acceleration_xy_TCP(t,3)));   
    end
    lastNull = [0 0 0]
    acceleration_xy_TCP(end+1,:) = lastNull(1,:)
    time_Spline(end+1) = summeZeitIntervalle

    acceleration_xy_TCP_kurz = [];

    for dddd = 1:alleXte:length(acceleration_xy_TCP)        
        acceleration_xy_TCP_kurz(end+1,:) = acceleration_xy_TCP(dddd,:);
    end
    
    acceleration_xy_TCP_kurz(end+1,:) = [0 0 0];
    
    [aaa, bbb] = size(acceleration_xy_TCP_kurz);

    zeitintervall_kurz = summeZeitIntervalle/(aaa-1);

    zeit_kurz = [];

    for ssdsd = 1:length(acceleration_xy_TCP_kurz)
        if ssdsd == 1
            zeit_kurz(ssdsd) = 0;
        else
            zeit_kurz(ssdsd) = zeit_kurz(ssdsd-1) + zeitintervall_kurz;
        end  
    end 

    optimized_translational_values(1,:) = zeitintervall_kurz;

    

    acceleration_xy_TCP = acceleration_xy_TCP_kurz; 
    time_Spline = zeit_kurz;

   [aa, bb] = size(punkteVersetzen)

    for ssss = 1:aa
        acceleration_xy_TCP(punkteVersetzen(ssss,1), punkteVersetzen(ssss,2)) = punkteVersetzen(ssss,3);
    end

    save('input/InputSOBGB_haendisch_DG_determinationAccelerationTCP.mat');

    figure
    hold on
    plot(time_Spline,acceleration_xy_TCP(:,1))
    plot(time_Spline,acceleration_xy_TCP(:,2))
    plot(time_Spline,acceleration_xy_TCP(:,3))
    legend('Acc_X_TCP','Acc_Y_TCP','Acc_Z_TCP')
%     figure;
%     hold on
% %     plot(time_Spline,orientation_xy(:,1))
%     plot(time_Spline,orientation_xy(:,2))
%     plot(time_Spline,orientation_xy(:,3))
%     legend('orientation_xy_X_TCP','orientation_xy_Y_TCP','orientation_xy_Z_TCP')
end

