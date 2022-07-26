function [eulerZYX,Beschl_xyz] = generate_Z_Rot(optimization_values, axesPointConfigs, splineDiscretization,visualize,optiParam)
%% Einlesen von Variablen
    
    %Erstellen von Leeren Zero Arrays
    achsstellungen = axesPointConfigs.';
    optimization_values = [zeros(size(optimization_values,1),1),optimization_values];    
    wayPoints = [];
    zRots = [];
    tpts = [];

    %Variabeln um "optimirung" von rotZ zu steuern
    offsetToBorder = optiParam(1);
    minIter = optiParam(2);
    maxiter = optiParam(3);
    span_to_Smooth = optiParam(4); %As value from 0 to 1 
    stepsize = optiParam(5);
    widhtStuetzp = optiParam(6);
    grenzSchwappY = optiParam(7);

    
    for p = 1:height(achsstellungen)
        [tcppunkt, eul, ~, ~, ~] = vorwaertskinematik(achsstellungen(p,:));
        wayPoints(:,p) = tcppunkt(:,1);
        zRots(:,p) = eul(1,1);
    end
            
    %Berechnen der Zeitstempel aus den Zeitintervallen aus der Optimirung
    for i = 1:size(optimization_values,2)
        tpts(i) = sum(optimization_values(1, 1:i));
    end

    %Vektor mit den Timestamps
    tvec = 0:0.0005:tpts(1,end); % Uhrsprüngliche Inkrementierung 0.01

    %Berechnen der indexe der Stützpunkte und der zeitlichen Varianz
    [ind_tpts_in_tVec,~] = dsearchn(transpose(tvec),transpose(tpts));
        
%% Fallunterscheidung ob z Berücksichtigt wird    
    if size(optimization_values,1) == 5
        %X und Y geschwindigkeit bzw beschleunigung
        VelocityBoundaryCondition_x = [optimization_values(2,:)];
        VelocityBoundaryCondition_y = [optimization_values(3,:)];    
        AccelerationBoundaryCondition_x = [optimization_values(4,:)];
        AccelerationBoundaryCondition_y = [optimization_values(5,:)];

        [q_x,qd_x,qdd_x,~] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x);
        [q_y,qd_y,qdd_y,~] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y); 
        q_z = zeros(size(q_y));
        qd_z = zeros(size(qd_y));
        qdd_z = zeros(size(qdd_y));
    else
        %z geschwindigkeit bzw beschleunigung
        VelocityBoundaryCondition_x = [optimization_values(2,:)];
        VelocityBoundaryCondition_y = [optimization_values(3,:)];
        VelocityBoundaryCondition_z = [optimization_values(4,:)];
        AccelerationBoundaryCondition_x = [optimization_values(5,:)];
        AccelerationBoundaryCondition_y = [optimization_values(6,:)];
        AccelerationBoundaryCondition_z = [optimization_values(7,:)];

        [q_x,qd_x,qdd_x,~] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x);
        [q_y,qd_y,qdd_y,~] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y);
        [q_z,qd_z,qdd_z,~] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_z,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_z);
    end
    
    q_xyz = [transpose(q_x), transpose(q_y), transpose(q_z)];
    qd_xyz = [transpose(qd_x), transpose(qd_y), transpose(qd_z)];
    qdd_xyz = [transpose(qdd_x), transpose(qdd_y), transpose(qdd_z)];
  
    %% Rot aus geschwindigkeit
    z_Rot_V = atan2d(qd_xyz(:,2),qd_xyz(:,1));
    z_Rot_V(1,1) = z_Rot_V(2,1);
    z_Rot_Acc = atan2d(qdd_xyz(:,2),qdd_xyz(:,1));
    z_Rot_Acc(1,1) = z_Rot_Acc(2,1);
    z_Rot_Acc = wrapTo_negativ_360_To_360(z_Rot_Acc);  
    z_Rot_Acc_unfilterd = wrapTo_negativ_360_To_360(z_Rot_Acc);
    z_Rot_Acc_unfilterd_2 = z_Rot_Acc_unfilterd+180;
    z_Rot_Acc_unfilterd_3 = z_Rot_Acc_unfilterd-180;
    abs_Acc = sqrt(qdd_xyz(:,1).^2+qdd_xyz(:,2).^2);

%% Berechnet gültige Rotationen (upper and lower boundrys)
    acc_TCP_XY = [];
    winkelGrenz = [];
    lb_1 = [];
    ub_1 = [];
    istNaN = 0;
    warNanN = 0;

%     wasNum = 0;
    for i = 1:size(z_Rot_Acc,1)
        acc_Max_XY(i,1:3) = transpose(RotationDegUmZ(-z_Rot_Acc(i,1))*transpose(qdd_xyz(i,:)));
        winkelGrenz(i,1) = asind(2.5/abs_Acc(i,1));

        if abs_Acc(i,1) <= grenzSchwappY
            lb_1(i,1) = NaN;
            ub_1(i,1) = NaN;
            istNaN = 1;
        elseif abs_Acc(i,1) > grenzSchwappY        
            lb_1(i,1) = z_Rot_Acc(i,1)-winkelGrenz(i,1);
            ub_1(i,1) = z_Rot_Acc(i,1)+winkelGrenz(i,1)-180;
            istNaN = 0;            
        end

        %Conect ub with lb at optimal +90°
        if i > 1 && ~(istNaN == warNanN) && warNanN == 0
            lb_1(i-1,1) = z_Rot_Acc(i,1)-90;
            ub_1(i-1,1) = z_Rot_Acc(i,1)+90-180;
        elseif i > 1 && ~(istNaN == warNanN) && warNanN == 1
            lb_1(i,1) = z_Rot_Acc(i,1)-90;
            ub_1(i,1) = z_Rot_Acc(i,1)+90-180;
        end

        warNanN = istNaN;
    end

    

    %% X intervall in dem eine Limitation besteht
    timeStmpd_Start = [];
    timeStmpd_End = [];

    timeStmpd_Start_ind = [];
    timeStmpd_End_ind = [];
    bubbleNum = 1;
    IndexBuble = [];
    
    for y = 2:size(ub_1,1)
        if isnan(lb_1(y-1,1)) && ~isnan(lb_1(y,1)) 
            timeStmpd_Start(end+1,1) = tvec(1,y);
            timeStmpd_Start_ind(end+1,1) = y;
            IndexBuble(bubbleNum,1) = y;
        elseif ~isnan(lb_1(y-1,1)) && isnan(lb_1(y,1))
            timeStmpd_End(end+1,1) = tvec(1,y-1);
            timeStmpd_End_ind(end+1,1) = y-1;  
            IndexBuble(bubbleNum,2) = y-1;
            bubbleNum = bubbleNum+1;
        end
    end

    maxBuble = [];
    minBuble = [];
    
    for i = 1:size(IndexBuble,1)     
        maxBuble(i,1) = max(lb_1(IndexBuble(i,1):IndexBuble(i,2),1));
        minBuble(i,1) = min(ub_1(IndexBuble(i,1):IndexBuble(i,2),1));

        if minBuble(i,1) > 360
            lb_1(IndexBuble(i,1):IndexBuble(i,2),1) = lb_1(IndexBuble(i,1):IndexBuble(i,2),1) - 360;
            ub_1(IndexBuble(i,1):IndexBuble(i,2),1) = ub_1(IndexBuble(i,1):IndexBuble(i,2),1) - 360;

        elseif maxBuble(i,1) < -360
            lb_1(IndexBuble(i,1):IndexBuble(i,2),1) = lb_1(IndexBuble(i,1):IndexBuble(i,2),1) + 360;
            ub_1(IndexBuble(i,1):IndexBuble(i,2),1) = ub_1(IndexBuble(i,1):IndexBuble(i,2),1) + 360;
        end

        if minBuble(i,1) > 180
            lb_1(IndexBuble(i,1):IndexBuble(i,2),1) = lb_1(IndexBuble(i,1):IndexBuble(i,2),1) - 180;
            ub_1(IndexBuble(i,1):IndexBuble(i,2),1) = ub_1(IndexBuble(i,1):IndexBuble(i,2),1) - 180;

        elseif maxBuble(i,1) < -180
            lb_1(IndexBuble(i,1):IndexBuble(i,2),1) = lb_1(IndexBuble(i,1):IndexBuble(i,2),1) + 180;
            ub_1(IndexBuble(i,1):IndexBuble(i,2),1) = ub_1(IndexBuble(i,1):IndexBuble(i,2),1) + 180;
        end
    end


%     lb_1 = wrapTo360(lb_1);   

    %% filter Lb und Ub sodass -180 < lb,ub < 180
    lb_2 = lb_1+180;
    ub_2 = ub_1+180;
    lb_3 = lb_1-180;
    ub_3 = ub_1-180;
    lb_4 = lb_1+360;
    ub_4 = ub_1+360;
    lb_5 = lb_1-360;
    ub_5 = ub_1-360;

% % %     ub_1 = wrapTo180(ub_1);
% % %     lb_2 = wrapTo180(lb_2);
% % %     ub_2 = wrapTo180(ub_2);
% % %     lb_3 = wrapTo180(lb_3);
% % %     ub_3 = wrapTo180(ub_3);

    lb_1 = wrapTo_negativ_360_To_360(lb_1);
    ub_1 = wrapTo_negativ_360_To_360(ub_1);
    lb_2 = wrapTo_negativ_360_To_360(lb_2);
    ub_2 = wrapTo_negativ_360_To_360(ub_2);
    lb_3 = wrapTo_negativ_360_To_360(lb_3);
    ub_3 = wrapTo_negativ_360_To_360(ub_3);
    lb_4 = wrapTo_negativ_360_To_360(lb_4);
    ub_4 = wrapTo_negativ_360_To_360(ub_4);
    lb_5 = wrapTo_negativ_360_To_360(lb_5);
    ub_5 = wrapTo_negativ_360_To_360(ub_5);


%% Abschätzen der Maximalen Achsumorientierung
    

    
%% Berechnung inertialer bahn aus Stützpunkten und Geschwindigkeit
    for i = 1:size(ind_tpts_in_tVec,1)-1 
        %Berechne Lin
        steigung = (zRots(i+1)-zRots(i))/(tvec(ind_tpts_in_tVec(i+1))-tvec(ind_tpts_in_tVec(i)));
        offset = zRots(i)-steigung*(tvec(ind_tpts_in_tVec(i))-tvec(i));
        linsp = transpose(tvec(ind_tpts_in_tVec(i):ind_tpts_in_tVec(i+1)));
        StuetzRot(ind_tpts_in_tVec(i):ind_tpts_in_tVec(i+1),1) = (linsp*steigung)+offset;
    end
    
%% optimierung der Rotation
    acc_TCP_XY = [];
    violaton = 1;
    iter = 0;
       
    z_Rot = StuetzRot;
    z_Rot_V_unfilterd = z_Rot;

    wight = ones(size(tvec,2),1);

    while  violaton >= 1 
        %Zurücksetze/ updaten der Hilfsvariablen
        violaton = minIter-iter;
        iter = iter + 1;
        bestBorder = 0;
        currStuetzp = 1;
                
%Filter Loop, der Gültige werte erzwingt
        for i = 1:size(z_Rot,1)
            
            %Erzwingt, dass Knotenpunkte berücksichtigt werden
            if i == ind_tpts_in_tVec(currStuetzp) 

                %Intervallbreite um stützpunkt zu gewichten    
                if i == 1
                    minus = 0;
                    plus = round(widhtStuetzp);
    
                elseif i == size(z_Rot,1)
                    minus = -round(widhtStuetzp);
                    plus = 0;
                else
                    minus = -round(widhtStuetzp/2);
                    plus = round(widhtStuetzp/2);
                end

                if z_Rot(i,1) < zRots(currStuetzp) - offsetToBorder*4
                    mini = i+minus;
                    maxi = i+plus;
                    z_Rot(mini:maxi,1) = zRots(currStuetzp) - offsetToBorder;
                    wight(i,1) = 100;
                    violaton = violaton+1;
                elseif z_Rot(i,1) > zRots(currStuetzp) + offsetToBorder*4
                    z_Rot(i+minus:i+plus,1) = zRots(currStuetzp) + offsetToBorder;
                    wight(i,1) = 100;
                    violaton = violaton+1;
                end  
                
                currStuetzp = currStuetzp+1;             
            end
            
            acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(-z_Rot(i,1))*transpose(qdd_xyz(i,:)));
%Fallunterscheidung ob RotZ zu klein oder groß
            if abs(acc_TCP_XY(i,2)) > grenzSchwappY %-0.01 %Zur näheren grenze implementieren
                %Anzeigen, dass eine violation vorliegt
                violaton = violaton+1;
    
                if bestBorder == 0
                    %Finde nächstgelegene grenze
                    [~, index] = min([abs(ub_1(i,1)-z_Rot(i,1)) , abs(ub_2(i,1)-z_Rot(i,1)) , abs(ub_3(i,1)-z_Rot(i,1)) , abs(ub_4(i,1)-z_Rot(i,1)) , abs(ub_5(i,1)-z_Rot(i,1))...
                        abs(lb_1(i,1)-z_Rot(i,1)) , abs(lb_2(i,1)-z_Rot(i,1)) , abs(lb_3(i,1)-z_Rot(i,1)) , abs(lb_4(i,1)-z_Rot(i,1)) , abs(lb_5(i,1)-z_Rot(i,1))]);

                    if index == 1
                        z_Rot(i,1) = ub_1(i,1)-offsetToBorder;
                        bestBorder = 1;

                    elseif index == 2
                        z_Rot(i,1) = ub_2(i,1)-offsetToBorder;
                        bestBorder = 2;

                    elseif index == 3
                        z_Rot(i,1) = ub_3(i,1)-offsetToBorder;
                        bestBorder = 3;

                    elseif index == 4
                        z_Rot(i,1) = ub_4(i,1)-offsetToBorder;
                        bestBorder = 4;

                    elseif index == 5
                        z_Rot(i,1) = ub_5(i,1)+offsetToBorder;
                        bestBorder = 5;

                    elseif index == 6
                        z_Rot(i,1) = lb_1(i,1)+offsetToBorder;
                        bestBorder = 6;

                    elseif index == 7
                        z_Rot(i,1) = lb_2(i,1)+offsetToBorder;
                        bestBorder = 7;

                    elseif index == 8
                        z_Rot(i,1) = lb_3(i,1)+offsetToBorder;
                        bestBorder = 8;

                    elseif index == 9
                        z_Rot(i,1) = lb_4(i,1)+offsetToBorder;
                        bestBorder = 9;

                    elseif index == 10
                        z_Rot(i,1) = lb_5(i,1)+offsetToBorder;
                        bestBorder = 10;

                    end
                    
                %widerholtes setzen der nächstgelegene grenze
                elseif bestBorder == 1
                    z_Rot(i,1) = ub_1(i,1)-offsetToBorder;

                elseif bestBorder == 2
                    z_Rot(i,1) = ub_2(i,1)-offsetToBorder;

                elseif bestBorder == 3
                    z_Rot(i,1) = ub_3(i,1)-offsetToBorder;

                elseif bestBorder == 4
                    z_Rot(i,1) = ub_4(i,1)-offsetToBorder;

                elseif bestBorder == 5
                    z_Rot(i,1) = ub_5(i,1)-offsetToBorder;

                elseif bestBorder == 6
                    z_Rot(i,1) = lb_1(i,1)+offsetToBorder;

                elseif bestBorder == 7
                    z_Rot(i,1) = lb_2(i,1)+offsetToBorder;

                elseif bestBorder == 8
                    z_Rot(i,1) = lb_3(i,1)+offsetToBorder;  

                elseif bestBorder == 9
                    z_Rot(i,1) = lb_4(i,1)+offsetToBorder; 

                elseif bestBorder == 10
                    z_Rot(i,1) = lb_5(i,1)+offsetToBorder; 

                end
            end 

            %Minimalabstand zu den grenzen als 1/2 offsetToBorder
            
        end
        
%smoothing Funktion die nicht zwangsweise gültige lösung liefert Dafür werden zunächst stützpunkte gelegt, wodurch anschlißend ein Spline gelegt wird
        if violaton >= 1
            smoothRot = smooth(z_Rot,span_to_Smooth,'lowess');
            fitObj = fit(transpose(tvec(1,1:stepsize:end)),smoothRot(1:stepsize:end,1),'smoothingspline','Weights',wight(1:stepsize:end,1),'SmoothingParam',1);
            z_Rot = fitObj(transpose(tvec));
            z_Rot = smoothRot;
        end

        if iter == maxiter
            violaton = 0;
            disp("Max iteration reaced, "+ num2str(violaton) + "left!" )
        end
        %Anzeigen der Aktuellen Iterationsnummer        
        disp("Rot_Z Iteration: "+ num2str(iter) )
    end

    z_Rot = wrapTo_negativ_360_To_360(z_Rot);


    %Debug
    f = sqrt(acc_Max_XY(:,1).^2+acc_Max_XY(:,2).^2);
    g =  sqrt(acc_TCP_XY(:,1).^2+acc_TCP_XY(:,2).^2);
    test_Rot = f- g;
    %calc yAccTCP

    for m = 1:size( acc_TCP_XY,1)
         acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(-z_Rot(i,1))*transpose(qdd_xyz(i,:)));
    end


 
%% Erstellung der Output Variablen mit splineDiscretization vielen Punkten
    %Kartesische koord. Position
    linSpacce_ = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), linSpacce_);
    lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), linSpacce_);
    lin_yy_x_z = spline(tvec(1, :), q_xyz(:, 3), linSpacce_);

    %Beschleunigungsvektor (nur für visueller debug)
    acc_x = spline(tvec(1, :), qdd_xyz(:, 1), linSpacce_);
    acc_y = spline(tvec(1, :), qdd_xyz(:, 2), linSpacce_);
    acc_Z = spline(tvec(1, :), qdd_xyz(:, 3), linSpacce_);
    Beschl_xyz = [transpose(acc_x),transpose(acc_y),transpose(acc_Z)];
    Beschl_xyz(1,:) = qdd_xyz(2,:);

    RotZ = spline(tvec(1, :),z_Rot,linSpacce_);
    RotZ(1,1) = z_Rot(2,1);   
 
    %Output Variablen
    timeLine = linSpacce_; 
    eulerZYX = [transpose(RotZ),zeros(size(RotZ,2),1),zeros(size(RotZ,2),1)];
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y), transpose(lin_yy_x_z)];

   %% Visualisierung der Ergebnisse   
    if visualize == 1
        %Sprünge um 180° Filtern:
        for i = 2:size(z_Rot,1)-1
            if abs(z_Rot(i-1,1)-z_Rot(i,1)) > 170
                z_Rot(i,1) = NaN;
            end
            if abs(ub_1(i-1,1)-ub_1(i,1)) > 170
                ub_1(i,1) = NaN;
            end
            if abs(ub_2(i-1,1)-ub_2(i,1)) > 170
                ub_2(i,1) = NaN;
            end
            if abs(ub_3(i-1,1)-ub_3(i,1)) > 170
                ub_3(i,1) = NaN;
            end
            if abs(lb_1(i-1,1)-lb_1(i,1)) > 170
                lb_1(i,1) = NaN;
            end
            if abs(lb_2(i-1,1)-lb_2(i,1)) > 170
                lb_2(i,1) = NaN;
            end
            if abs(lb_3(i-1,1)-lb_3(i,1)) > 170
                lb_3(i,1) = NaN;
            end
            if abs(z_Rot_Acc_unfilterd(i-1,1)-z_Rot_Acc_unfilterd(i,1)) > 170
                z_Rot_Acc_unfilterd(i,1) = NaN;
            end
            if abs(z_Rot_V_unfilterd(i-1,1)-z_Rot_V_unfilterd(i,1)) > 170
                z_Rot_V_unfilterd(i,1) = NaN;
            end
            if abs(z_Rot_Acc_unfilterd_2(i-1,1)-z_Rot_Acc_unfilterd_2(i,1)) > 170
                z_Rot_Acc_unfilterd_2(i,1) = NaN;
            end        
        end

        %Erstellen einer Ui Figure und setzen der Axes Eigenschaften
        figure('units','normalized','outerposition',[0 0 1 1])
        x = axes;
        x.YLim = [-185,185];  
        xlabel('deg [°]')
        ylabel('Z-Rotation')
        hold on
        
        %Plots mit Legende

        
        
        plot(tvec, z_Rot,'LineWidth',2.5)
        plot(tvec, ub_1,'r','LineWidth',1.5)
        plot(tvec, lb_1,'c','LineWidth',1.5)
        plot(tvec, z_Rot_Acc_unfilterd,'--g','LineWidth',1.5)
        plot(tvec, z_Rot_V_unfilterd,'--m') 

        alphaBuble = 0.3;%Transparenz der ungültigen Bereiche        
        for i = 1:size(IndexBuble,1)
            times1 = [transpose(tvec(1,IndexBuble(i,1):IndexBuble(i,2)));transpose(flip(tvec(1,IndexBuble(i,1):IndexBuble(i,2))))];
            borders1 = [lb_1(IndexBuble(i,1):IndexBuble(i,2),1);flip(ub_1(IndexBuble(i,1):IndexBuble(i,2),1))];
            filld = fill(times1,borders1,'r','FaceAlpha',alphaBuble);
            uistack(filld,"bottom");

            times2 = [transpose(tvec(1,IndexBuble(i,1):IndexBuble(i,2)));transpose(flip(tvec(1,IndexBuble(i,1):IndexBuble(i,2))))];
            borders2 = [lb_2(IndexBuble(i,1):IndexBuble(i,2),1);flip(ub_2(IndexBuble(i,1):IndexBuble(i,2),1))];
            fill(times2,borders2,'r','FaceAlpha',alphaBuble)

            times3 = [transpose(tvec(1,IndexBuble(i,1):IndexBuble(i,2)));transpose(flip(tvec(1,IndexBuble(i,1):IndexBuble(i,2))))];
            borders3 = [lb_3(IndexBuble(i,1):IndexBuble(i,2),1);flip(ub_3(IndexBuble(i,1):IndexBuble(i,2),1))];
            fill(times3,borders3,'r','FaceAlpha',alphaBuble)

            times4 = [transpose(tvec(1,IndexBuble(i,1):IndexBuble(i,2)));transpose(flip(tvec(1,IndexBuble(i,1):IndexBuble(i,2))))];
            borders4 = [lb_4(IndexBuble(i,1):IndexBuble(i,2),1);flip(ub_4(IndexBuble(i,1):IndexBuble(i,2),1))];
            fill(times4,borders4,'r','FaceAlpha',alphaBuble)

            times5 = [transpose(tvec(1,IndexBuble(i,1):IndexBuble(i,2)));transpose(flip(tvec(1,IndexBuble(i,1):IndexBuble(i,2))))];
            borders5 = [lb_5(IndexBuble(i,1):IndexBuble(i,2),1);flip(ub_5(IndexBuble(i,1):IndexBuble(i,2),1))];
            fill(times5,borders5,'r','FaceAlpha',alphaBuble)

        end
      
        %Widerholungen um 180° bzw 360° verschoben
        plot(tvec, ub_2,'r','LineWidth',1.5)
        plot(tvec, lb_2,'c','LineWidth',1.5)
        plot(tvec, ub_3,'r','LineWidth',1.5)
        plot(tvec, lb_3,'c','LineWidth',1.5) 
        plot(tvec, ub_4,'r','LineWidth',1.5)
        plot(tvec, lb_4,'c','LineWidth',1.5) 
        plot(tvec, ub_5,'r','LineWidth',1.5)
        plot(tvec, lb_5,'c','LineWidth',1.5) 
        plot(tvec, z_Rot_Acc_unfilterd_2,'--g','LineWidth',1.5)
        plot(tvec, z_Rot_Acc_unfilterd_3,'--g','LineWidth',1.5)
         
        %Plotten der Ausgangsgröße
        plot(tpts,zRots,'o')

        %x, yLines zur visaulisirung von bereichen
        xline(timeStmpd_Start,'--','Untere X Grenze','LabelHorizontalAlignment','left')
        xline(timeStmpd_End,'--','Obere X Grenze','LabelHorizontalAlignment','right')
        yline(180, '--b','180°','LabelVerticalAlignment','bottom','LabelHorizontalAlignment','left')
        yline(-180, '--b','-180°','LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
        yline(360, '--b','360°','LabelVerticalAlignment','bottom','LabelHorizontalAlignment','left')
        yline(-360, '--b','-360°','LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
        legend('Rot_Z','ub','lb','optimal','Rot_Z unfilterd')
        
        %Plot Roboterbahn und Beschleunigung in 3d
        roboPath = figure;
        robiAx = axes(roboPath);     
        hold(robiAx,"on");
        for i = 1:2:size(Position_xyz,1)
            plotCoord_syst(0.15,Position_xyz(i,1),Position_xyz(i,2),Position_xyz(i,3),eulerZYX(i,1),robiAx)
        end
        plot3(robiAx,Position_xyz(:,1),Position_xyz(:,2),Position_xyz(:,3),'-o','Color','g','MarkerSize',4,'MarkerFaceColor','auto');
        quiver3(robiAx,Position_xyz(:,1),Position_xyz(:,2),Position_xyz(:,3),Beschl_xyz(:,1),Beschl_xyz(:,2),Beschl_xyz(:,3),"Color","r","AutoScale","on");
        arr = [(robiAx.XLim(1,2)-robiAx.XLim(1,1)),(robiAx.YLim(1,2)-robiAx.YLim(1,1)),(robiAx.ZLim(1,2)-robiAx.ZLim(1,1))];
        limWidth = max(arr);
        robiAx.XLim = [robiAx.XLim(1,1),robiAx.XLim(1,1)+limWidth];
        robiAx.YLim = [robiAx.YLim(1,1),robiAx.XLim(1,1)+limWidth];
        robiAx.ZLim = [0,limWidth];
        robiAx.View = [35,35];
        robiAx.XGrid = "on";
        robiAx.YGrid = "on";
        robiAx.ZGrid = "on";
    end                
end