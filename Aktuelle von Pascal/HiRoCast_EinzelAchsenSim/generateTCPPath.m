function [Position_xyz, timeLine] = generateTCPPath(optimization_values, wayPoints, splineDiscretization, visualizeTCPPath, min_values, max_values, jerkBoundaries)
    [breite, hoehe] = size(optimization_values);
    
% % %     tvec = 0:0.01:optimization_values(1, end);
% % %     tpts = optimization_values(1, 1:end);

% % %     %Berechnen der indexe der Stützpunkte und der zeitlichen Varianz
% % %     [ind_tpts_in_tVec,dist] = dsearchn(transpose(tvec),transpose(tpts));
% % %     
% % %     
% % % %     for i = 1:size(tpts,2)
% % % %        [ind_tpts_in_tVec,dist] = dsearchn(tpts(1,i),tvec);
% % % %     end
% % % 
% % %     VelocityBoundaryCondition_x = [0 optimization_values(2, 1) 0];
% % %     VelocityBoundaryCondition_y = [0 optimization_values(2, 2) 0];
% % %     VelocityBoundaryCondition_z = [0 optimization_values(2, 3) 0];
% % % 
% % %     AccelerationBoundaryCondition_x = [0 optimization_values(3, 1) 0];
% % %     AccelerationBoundaryCondition_y = [0 optimization_values(3, 2) 0];
% % %     AccelerationBoundaryCondition_z = [0 optimization_values(3, 3) 0];
% % % 
% % %     [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x);
% % %     [q_y,qd_y,qdd_y,pp_y] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y);
% % %     [q_z,qd_z,qdd_z,pp_z] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_z,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_z);
% % % 
% % %     q_xyz = [transpose(q_x), transpose(q_y), transpose(q_z)];
% % %     qd_xyz = [transpose(qd_x), transpose(qd_y), transpose(qd_z)];
% % %     qdd_xyz = [transpose(qdd_x), transpose(qdd_y), transpose(qdd_z)];
% % %     pp_xyz = [transpose(pp_x), transpose(pp_y), transpose(pp_z)];
% % % 
% % %     %Rot aus geschwindigkeit
% % %     z_Rot_V = atan2d(qd_xyz(:,2),qd_xyz(:,1));
% % %     z_Rot_V(1,1) = z_Rot_V(2,1);
% % % %     z_Rot_V_unfilterd = z_Rot_V;
% % % %     z_Rot_V = zeros(size(z_Rot_V,1),1)+15;
% % %     z_Rot_Acc = atan2d(qdd_xyz(:,2),qdd_xyz(:,1));
% % %     z_Rot_Acc(1,1) = z_Rot_Acc(2,1);
% % %     z_Rot_Acc_unfilterd = z_Rot_Acc;
% % %     z_Rot_Acc_unfilterd_2 = wrapTo180(z_Rot_Acc+180);
% % %     abs_Acc = sqrt(qdd_xyz(:,1).^2+qdd_xyz(:,2).^2);
% % % 
% % %     %Berechnet gültige Rotationen (upper and lower boundrys)
% % %     acc_TCP_XY = [];
% % %     winkelGrenz = [];
% % %     lb_1 = [];
% % %     ub_1 = [];
% % %     lb_2 = [];
% % %     ub_2 = [];
% % %     for i = 1:size(z_Rot_Acc,1)
% % %         acc_Max_XY(i,1:3) = transpose(RotationDegUmZ(-z_Rot_Acc(i,1))*transpose(qdd_xyz(i,:)));
% % %         winkelGrenz(i,1) = asind(2.5/abs_Acc(i,1));
% % %         if abs_Acc(i,1) <= 2.5
% % %             lb_1(i,1) = NaN;
% % %             ub_1(i,1) = NaN;
% % %             lb_2(i,1) = NaN;
% % %             ub_2(i,1) = NaN;
% % %             
% % %         elseif abs_Acc(i,1) > 2.5        
% % %             lb_1(i,1) = z_Rot_Acc(i,1)-winkelGrenz(i,1);
% % %             ub_1(i,1) = z_Rot_Acc(i,1)+winkelGrenz(i,1);
% % %             lb_2(i,1) = z_Rot_Acc(i,1)-winkelGrenz(i,1)-180;
% % %             ub_2(i,1) = z_Rot_Acc(i,1)+winkelGrenz(i,1)-180;
% % %         end
% % %     end
% % % 
% % %     % filter Lb und Ub sodass -180 < lb,ub < 180
% % %     lb_1 = wrapTo180(lb_1);
% % %     ub_1 = wrapTo180(ub_1);
% % %     lb_2 = wrapTo180(lb_2);
% % %     ub_2 = wrapTo180(ub_2);
% % % 
% % %     %X intervall in dem eine Limitation besteht
% % %     timeStmpd_Start = [];
% % %     timeStmpd_End = [];
% % %     for y = 2:size(ub_1,1)
% % %         if isnan(lb_1(y-1,1)) && ~isnan(lb_1(y,1)) 
% % %             timeStmpd_Start(end+1,1) = tvec(1,y);
% % %         elseif ~isnan(lb_1(y-1,1)) && isnan(lb_1(y,1))
% % %             timeStmpd_End(end+1,1) = tvec(1,y-1);
% % %         end
% % %     end
% % %     
% % %     %Berechne beschleunigungen im vorläufigen TCP/ Gleichzeitige optimierung der Rotation
% % %     acc_TCP_XY = [];
% % %     violaton = 1;
% % %     iter = 0;
% % %     minIter = 20;
% % % 
% % %     %Berechnung inertialer bahn aus Stützpunkten und Geschwindigkeit
% % %     for i = 1:size(ind_tpts_in_tVec,1)-1 
% % %         %Berechne Lin
% % %         steigung = (zRots(i+1)-zRots(i))/(tvec(ind_tpts_in_tVec(i+1))-tvec(ind_tpts_in_tVec(i)));
% % %         offset = zRots(i)-steigung*(tvec(ind_tpts_in_tVec(i))-tvec(i));
% % %         linsp = transpose(tvec(ind_tpts_in_tVec(i):ind_tpts_in_tVec(i+1)));
% % %         StuetzRot(ind_tpts_in_tVec(i):ind_tpts_in_tVec(i+1),1) = (linsp*steigung)+offset;
% % %     end
% % % 
% % %     z_Rot = StuetzRot;
% % %     z_Rot_V_unfilterd = z_Rot;
% % % 
% % %     %Variabeln um "optimirung" von rotZ zu steuern
% % %     stepsize = 5;
% % %     offsetToBorder = 5;%Positive Variabel !!!!!
% % %     wight = ones(size(tvec,2),1);
% % % 
% % %     while  violaton >= 1 
% % %         %Zurücksetze/ updaten der Hilfsvariablen
% % %         violaton = minIter-iter;
% % %         iter = iter + 1;
% % %         bestBorder = 0;
% % %         currStuetzp = 1;
% % %                 
% % %         %Filter Loop, der Gültige werte erzwingt
% % %         for i = 1:size(z_Rot,1)
% % % 
% % %             %Erzwingt, dass Knotenpunkte berücksichtigt werden
% % %             if i == ind_tpts_in_tVec(currStuetzp) 
% % % 
% % %                 %Intervallbreite um stützpunkt zu gewichten    
% % %                 if i == 1
% % %                     minus = 0;
% % %                     plus = 10;
% % %     
% % %                 elseif i == size(z_Rot,1)
% % %                     minus = -10;
% % %                     plus = 0;
% % %                 else
% % %                     minus = -5;
% % %                     plus = 5;
% % %                 end
% % % 
% % %                 if z_Rot(i,1) < zRots(currStuetzp) - offsetToBorder
% % %                     z_Rot(i+minus:i+plus,1) = zRots(currStuetzp) - offsetToBorder;
% % %                     wight(i,1) = 100;
% % %                     violaton = violaton+1;
% % %                 elseif z_Rot(i,1) > zRots(currStuetzp) + offsetToBorder
% % %                     z_Rot(i+minus:i+plus,1) = zRots(currStuetzp) + offsetToBorder;
% % %                     wight(i,1) = 100;
% % %                     violaton = violaton+1;
% % %                 end  
% % %                 
% % %                 currStuetzp = currStuetzp+1;
% % %                 
% % %             end
% % %             
% % %             acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(-z_Rot(i,1))*transpose(qdd_xyz(i,:)));
% % %             %Fallunterscheidung ob RotZ zu klein oder groß
% % %             if abs(acc_TCP_XY(i,2)) > 2.501 %Zur näheren grenze implementieren
% % %                 %Anzeigen, dass eine violation vorliegt
% % %                 violaton = violaton+1;
% % %     
% % %                 if bestBorder == 0
% % %                     %Finde nächstgelegene grenze
% % %                     [~, index] = min([abs(ub_1(i,1)-z_Rot(i,1)),abs(ub_2(i,1)-z_Rot(i,1)),...
% % %                         abs(lb_1(i,1)-z_Rot(i,1)),abs(lb_2(i,1)-z_Rot(i,1))]);
% % % 
% % %                     if index == 1
% % %                         z_Rot(i,1) = ub_1(i,1)-offsetToBorder;
% % %                         bestBorder = 1;
% % % 
% % %                     elseif index == 2
% % %                         z_Rot(i,1) = ub_2(i,1)-offsetToBorder;
% % %                         bestBorder = 2;
% % % 
% % %                     elseif index == 3
% % %                         z_Rot(i,1) = lb_1(i,1)+offsetToBorder;
% % %                         bestBorder = 3;
% % % 
% % %                     elseif index == 4
% % %                         z_Rot(i,1) = lb_2(i,1)+offsetToBorder;
% % %                         bestBorder = 4;
% % %                     end
% % %                 %widerholtes setzen der nächstgelegene grenze
% % %                 elseif bestBorder == 1
% % %                     z_Rot(i,1) = ub_1(i,1)-offsetToBorder;
% % % 
% % %                 elseif bestBorder == 2
% % %                     z_Rot(i,1) = ub_2(i,1)-offsetToBorder;
% % % 
% % %                 elseif bestBorder == 3
% % %                     z_Rot(i,1) = lb_1(i,1)+offsetToBorder;
% % % 
% % %                 elseif bestBorder == 4
% % %                     z_Rot(i,1) = lb_2(i,1)+offsetToBorder;  
% % % 
% % %                 end
% % %             end          
% % %         end
% % % 
% % %         %smoothing Funktion die nicht zwangsweise gültige lösung liefert Dafür werden zunächst stützpunkte gelegt, wodurch anschlißend ein Spline gelegt wird
% % %         if violaton >= 1
% % %             fitObj = fit(transpose(tvec(1,1:stepsize:end)),z_Rot(1:stepsize:end,1),'smoothingspline','Weights',wight(1:stepsize:end,1));
% % %             z_Rot = fitObj(transpose(tvec));
% % %         end
% % %         %Anzeigen der Aktuellen Iterationsnummer        
% % %         disp("Rot_Z Iteration: "+ num2str(iter))
% % %     end
% % % 
% % %     %Debug
% % %     f = sqrt(acc_Max_XY(:,1).^2+acc_Max_XY(:,2).^2);
% % %     g =  sqrt(acc_TCP_XY(:,1).^2+acc_TCP_XY(:,2).^2);
% % %     test_Rot = f- g;
% % % 
% % % 
% % %  
% % %     %Erstellung der Variablen mit splineDiscretization vielen Punkten
% % %     %Kartesische koord. Position
% % %     linSpacce_ = linspace(0, tvec(1, end), splineDiscretization);
% % %     lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), linSpacce_);
% % %     lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), linSpacce_);
% % %     lin_yy_x_z = spline(tvec(1, :), q_xyz(:, 3), linSpacce_);
% % % 
% % %     %Beschleunigungsvektor (nur für visueller debug)
% % %     acc_x = spline(tvec(1, :), qdd_xyz(:, 1), linSpacce_);
% % %     acc_y = spline(tvec(1, :), qdd_xyz(:, 2), linSpacce_);
% % %     acc_Z = spline(tvec(1, :), qdd_xyz(:, 3), linSpacce_);
% % %     Beschl_xyz = [transpose(acc_x),transpose(acc_y),transpose(acc_Z)];
% % %     Beschl_xyz(1,:) = qdd_xyz(2,:);
% % % 
% % %     RotZ = spline(tvec(1, :),z_Rot,linSpacce_);
% % %     RotZ(1,1) = z_Rot(2,1);
% % % 
% % % % % %     %Berechnen der upper and lower boundry
% % % % % %     lb = zeros
% % % % % %     ub = 
% % % % % %     for i = 1:size(RotZ,2)
% % % % % %         acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(RotZ(1,i))*transpose(Beschl_xyz(i,:)));
% % % % % %         winkelGrenz = atand((2.5)*sign(acc_TCP_XY(i,2))/acc_TCP_XY(i,2)); 
% % % % % %     end
% % %     
% % % 
% % % % % %     %Berechne beschleunigungen im vorläufigen TCP/ Gleichzeitige
% % % % % %     %optimierung der Rotation
% % % % % %     acc_TCP_XY = [];
% % % % % %     sicherheit = 1;
% % % % % %     for i = 1:size(RotZ,2)
% % % % % %         acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(RotZ(1,i))*transpose(Beschl_xyz(i,:)));
% % % % % %         %Fallunterscheidung ob RotZ zu klein oder groß
% % % % % %         if acc_TCP_XY(i,2) > 0 && acc_TCP_XY(i,2)> 2.5*sicherheit
% % % % % %             RotZ(1,i) = RotZ(1,i) +acosd(2.5/(sqrt(acc_TCP_XY(i,2)^2+acc_TCP_XY(i,2)^2)));
% % % % % %         elseif acc_TCP_XY(i,2) < 0 && acc_TCP_XY(i,2) < -2.5*sicherheit 
% % % % % %              RotZ(1,i) = RotZ(1,i) -acosd(2.5/(sqrt(acc_TCP_XY(i,2)^2+acc_TCP_XY(i,2)^2)));
% % % % % %         end          
% % % % % %     end
% % % 
% % %     
% % % % % %     for i = 1:size(RotZ,2)
% % % % % %         acc_TCP_XY(i,1:3) = transpose(RotationDegUmZ(RotZ(1,i))*transpose(Beschl_xyz(i,:)));
% % % % % %         winkelIst = atand(acc_TCP_XY(i,2)/acc_TCP_XY(i,1));
% % % % % %         winkelGrenz = atand((2.5)*sign(acc_TCP_XY(i,2))/acc_TCP_XY(i,2));
% % % % % %         winkelZuGrenzwink = winkelGrenz-winkelIst;
% % % % % % %         RotZ(1,i) = RotZ(1,i) + winkelZuGrenzwink;
% % % % % %         if winkelZuGrenzwink > 0 && winkelGrenz > winkelIst
% % % % % %             continue
% % % % % %         elseif winkelZuGrenzwink < 0 && winkelGrenz < winkelIst
% % % % % %             RotZ(1,i) = RotZ(1,i) + winkelZuGrenzwink;
% % % % % %         elseif abs(winkelZuGrenzwink) > abs(2*winkelGrenz)
% % % % % %             RotZ(1,i) = RotZ(1,i) - winkelZuGrenzwink; 
% % % % % %         end    
% % % % % %     end
% % %     
% % %     %Visualisierung der Ergebnisse
% % %     if visualizeTCPPath
% % %         figure('units','normalized','outerposition',[0 0 1 1])
% % % 
% % %         subplot(4,1,1)
% % %         plot(tvec, q_xyz)
% % %         xline(tpts(2))
% % %         ylabel('Positions')
% % %         legend('X','Y','Z')
% % % 
% % %         subplot(4,1,2)
% % %         plot(tvec, qd_xyz)
% % %         ylabel('Velocities')
% % %         xline(tpts(2))
% % %         yline(max_values(2,1), '--b')
% % %         yline(min_values(2,1), '--b')
% % %         yline(max_values(2,2), '--r')
% % %         yline(min_values(2,2), '--r')
% % %         yline(max_values(2,3), '--y')
% % %         yline(min_values(2,3), '--y')
% % %         legend('X','Y','Z') 
% % % 
% % %         subplot(4,1,3)        
% % %         plot(tvec, qdd_xyz)
% % %         ylabel('acceleration')
% % %         xline(tpts(2))
% % %         yline(max_values(3,1), '--b')
% % %         yline(min_values(3,1), '--b')
% % %         yline(max_values(3,2), '--r')
% % %         yline(min_values(3,2), '--r')
% % %         yline(max_values(3,3), '--y')
% % %         yline(min_values(3,3), '--y')
% % %         legend('X','Y','Z') 
% % % 
% % %         qddd_xyz = diff(qdd_xyz);
% % %         qddd_xyz(end+1,:) = qddd_xyz(end,:);
% % %         subplot(4,1,4)
% % %         plot(tvec, qddd_xyz)
% % %         ylabel('Jerk')
% % %         xline(tpts(2))
% % %         yline(jerkBoundaries, '--b')
% % %         yline(-jerkBoundaries, '--b')
% % %         legend('X','Y','Z') 
% % % 
% % %         figure('units','normalized','outerposition',[0 0 1 1])
% % %         x = axes;
% % %         x.YLim = [-182,182];        
% % %         hold on
% % %         plot(tvec, z_Rot,'LineWidth',1)
% % %         plot(tvec, ub_1,'r','LineWidth',0.7)
% % %         plot(tvec, lb_1,'c','LineWidth',0.7)
% % %         plot(tvec, z_Rot_Acc_unfilterd,'g','LineWidth',0.7)
% % %         plot(tvec, z_Rot_V_unfilterd,'--m')
% % %         plot(tvec, ub_2,'r','LineWidth',0.7)
% % %         plot(tvec, lb_2,'c','LineWidth',0.7) 
% % %         plot(tpts,zRots,'o')
% % %         plot(tvec, z_Rot_Acc_unfilterd_2,'g','LineWidth',0.7)
% % %         xline(timeStmpd_Start,'--','Untere X Grenze','LabelHorizontalAlignment','left')
% % %         xline(timeStmpd_End,'--','Obere X Grenze','LabelHorizontalAlignment','right')
% % %         yline(180, '--b','180°','LabelVerticalAlignment','bottom','LabelHorizontalAlignment','left')
% % %         yline(-180, '--b','-180°','LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
% % %         xlabel('deg [°]')
% % %         ylabel('Z-Rotation')
% % %         legend('Rot_Z','ub','lb','optimal','Rot_Z unfilterd')
% % % 
% % %         
% % %     end
% % %     timeLine = linSpacce_; 
% % %     eulerZYX = [transpose(RotZ),zeros(size(RotZ,2),1),zeros(size(RotZ,2),1)];
% % %     Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y), transpose(lin_yy_x_z)];
% % % 
% % %     %Plot Roboterbahn und Beschleunigung in 3d
% % %     roboPath = figure;
% % %     robiAx = axes(roboPath);
% % %  
% % %     hold(robiAx,"on");
% % %     plot3(robiAx,Position_xyz(:,1),Position_xyz(:,2),Position_xyz(:,3),'-o','Color','g','MarkerSize',4,'MarkerFaceColor','auto');
% % %     quiver3(robiAx,Position_xyz(:,1),Position_xyz(:,2),Position_xyz(:,3),Beschl_xyz(:,1),Beschl_xyz(:,2),Beschl_xyz(:,3),"Color","r","AutoScale","on");
% % %     arr = [(robiAx.XLim(1,2)-robiAx.XLim(1,1)),(robiAx.YLim(1,2)-robiAx.YLim(1,1)),(robiAx.ZLim(1,2)-robiAx.ZLim(1,1))];
% % %     limWidth = max(arr);
% % %     robiAx.XLim = [robiAx.XLim(1,1),robiAx.XLim(1,1)+limWidth];
% % %     robiAx.YLim = [robiAx.YLim(1,1),robiAx.XLim(1,1)+limWidth];
% % %     robiAx.ZLim = [0,limWidth];
% % %     robiAx.View = [35,35];
% % %     robiAx.XGrid = "on";
% % %     robiAx.YGrid = "on";
% % %     robiAx.ZGrid = "on";
% % % end

if breite == 7
   for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)];

    %Berechnen der indexe der Stützpunkte und der zeitlichen Varianz
    [ind_tpts_in_tVec,dist] = dsearchn(transpose(tvec),transpose(tpts));

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]
    VelocityBoundaryCondition_y = [0, optimization_values(3,:)]
    VelocityBoundaryCondition_z = [0, optimization_values(4,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(5,:)]
    AccelerationBoundaryCondition_y = [0, optimization_values(6,:)]
    AccelerationBoundaryCondition_z = [0, optimization_values(7,:)]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)
    [q_y,qd_y,qdd_y,pp_y] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y)
    [q_z,qd_z,qdd_z,pp_z] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_z,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_z)

    q_xyz = [transpose(q_x), transpose(q_y), transpose(q_z)];
    qd_xyz = [transpose(qd_x), transpose(qd_y), transpose(qd_z)];
    qdd_xyz = [transpose(qdd_x), transpose(qdd_y), transpose(qdd_z)];
    pp_xyz = [transpose(pp_x), transpose(pp_y), transpose(pp_z)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);

    lin_xx_x_y = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), lin_xx_x_y);

    lin_xx_x_z = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_z = spline(tvec(1, :), q_xyz(:, 3), lin_xx_x_z);

    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(2,1), 'b')
        yline(min_values(2,1), 'b')
        yline(max_values(2,2), 'r')
        yline(min_values(2,2), 'r')
        yline(max_values(2,3), 'y')
        yline(min_values(2,3), 'y')
        legend('X','Y','Z') 

        subplot(4,1,3)        
        plot(tvec, qdd_xyz)
        xlabel('t')
        ylabel('acceleration')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(3,1), 'b')
        yline(min_values(3,1), 'b')
        yline(max_values(3,2), 'r')
        yline(min_values(3,2), 'r')
        yline(max_values(3,3), 'y')
        yline(min_values(3,3), 'y')
        legend('X','Y','Z') 

        qddd_xyz = diff(qdd_xyz);
        qddd_xyz(end+1,:) = qddd_xyz(end,:);
        subplot(4,1,4)
        plot(tvec, qddd_xyz)
        xlabel('t')
        ylabel('Jerk')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y), transpose(lin_yy_x_z)];

elseif breite == 5
       for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]
    VelocityBoundaryCondition_y = [0, optimization_values(3,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(4,:)]
    AccelerationBoundaryCondition_y = [0, optimization_values(5,:)]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(1,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)
    [q_y,qd_y,qdd_y,pp_y] = quinticpolytraj(wayPoints(2,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_y,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_y)

    q_xyz = [transpose(q_x), transpose(q_y)];
    qd_xyz = [transpose(qd_x), transpose(qd_y)];
    qdd_xyz = [transpose(qdd_x), transpose(qdd_y)];
    pp_xyz = [transpose(pp_x), transpose(pp_y)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);

    lin_xx_x_y = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_y = spline(tvec(1, :), q_xyz(:, 2), lin_xx_x_y);

    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(2,1), 'b')
        yline(min_values(2,1), 'b')
        yline(max_values(2,2), 'r')
        yline(min_values(2,2), 'r')
        yline(max_values(2,3), 'y')
        yline(min_values(2,3), 'y')
        legend('X','Y','Z') 

        subplot(4,1,3)        
        plot(tvec, qdd_xyz)
        xlabel('t')
        ylabel('acceleration')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(3,1), 'b')
        yline(min_values(3,1), 'b')
        yline(max_values(3,2), 'r')
        yline(min_values(3,2), 'r')
        yline(max_values(3,3), 'y')
        yline(min_values(3,3), 'y')
        legend('X','Y','Z') 

        qddd_xyz = diff(qdd_xyz);
        qddd_xyz(end+1,:) = qddd_xyz(end,:);
        subplot(4,1,4)
        plot(tvec, qddd_xyz)
        xlabel('t')
        ylabel('Jerk')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x), transpose(lin_yy_x_y)];
elseif breite == 3
       for t = 1:length(optimization_values(1,:))
        if t == 1
            optimization_values(1,t) = optimization_values(1,t)
        else
            optimization_values(1,t) = optimization_values(1,t-1) + optimization_values(1,t)
        end
    end

    tvec = 0:0.03:optimization_values(1, end);
    tpts = [0, optimization_values(1,:)]

    VelocityBoundaryCondition_x = [0, optimization_values(2,:)]

    AccelerationBoundaryCondition_x = [0, optimization_values(3,:)]

    [q_x,qd_x,qdd_x,pp_x] = quinticpolytraj(wayPoints(3,:), tpts, tvec, "VelocityBoundaryCondition", VelocityBoundaryCondition_x,"AccelerationBoundaryCondition",AccelerationBoundaryCondition_x)

    q_xyz = [transpose(q_x)];
    qd_xyz = [transpose(qd_x)];
    qdd_xyz = [transpose(qdd_x)];
    pp_xyz = [transpose(pp_x)];
    
    
    lin_xx_x_x = linspace(0, tvec(1, end), splineDiscretization);
    lin_yy_x_x = spline(tvec(1, :), q_xyz(:, 1), lin_xx_x_x);


    if visualizeTCPPath
        figure('units','normalized','outerposition',[0 0 1 1])

        subplot(4,1,1)
        plot(tvec, q_xyz)
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        xlabel('t')
        ylabel('Positions')
        legend('X','Y','Z')

        subplot(4,1,2)
        plot(tvec, qd_xyz)
        xlabel('t')
        ylabel('Velocities')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(2,1), 'b')
        yline(min_values(2,1), 'b')
        yline(max_values(2,2), 'r')
        yline(min_values(2,2), 'r')
        yline(max_values(2,3), 'y')
        yline(min_values(2,3), 'y')
        legend('X','Y','Z') 

        subplot(4,1,3)        
        plot(tvec, qdd_xyz)
        xlabel('t')
        ylabel('acceleration')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(max_values(3,1), 'b')
        yline(min_values(3,1), 'b')
        yline(max_values(3,2), 'r')
        yline(min_values(3,2), 'r')
        yline(max_values(3,3), 'y')
        yline(min_values(3,3), 'y')
        legend('X','Y','Z') 

        qddd_xyz = diff(qdd_xyz);
        qddd_xyz(end+1,:) = qddd_xyz(end,:);
        subplot(4,1,4)
        plot(tvec, qddd_xyz)
        xlabel('t')
        ylabel('Jerk')
        for l = 2:length(tpts)-1
            xline(tpts(l))
        end
        yline(jerkBoundaries, 'b')
        yline(-jerkBoundaries, 'b')
        legend('X','Y','Z') 
    end
    timeLine = lin_xx_x_x; 
    Position_xyz = [transpose(lin_yy_x_x)];
end
 
end

