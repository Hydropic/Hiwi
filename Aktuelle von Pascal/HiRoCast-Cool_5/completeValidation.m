function [c,ceq] = completeValidation(timeintervals, base_points,regression,simulation_data)
    fprintf('Validation spline + base\n');
    %Überprüfung der Schwappbedingung und des Korridores
    %   Input: Basepoints, Computes spline, iterates over all basepoints (extra
    %   cases first and last point), sets constraints on forward kinematics
    %   Outputs: Constraint Arrays: sum of validation and special values at basepoints       
    
    %Leere Constraints erzeugen
    c = [];
    ceq = [];
    
    schritte = 50; %voher 50

    %Um plotten des Roboters zu ermöglichen ausführen
    %KSetUp
   
    % Setting variance for acceleration
    variance_z = 5;%in Deg
    
    % Anpassen der vNamen in der Tabelle mit den Fluiddaten   
    simulation_data.Properties.VariableNames = {'rotationX','rotationY','rotationZ','directionX','directionY','directionZ','horizChange','vertChange','acceleration'};
        
    % Creating empty table-arrays
    directionX = [];
    directionY = [];
    directionZ = [];
    Acceleration = [];
    EulerZYX = [];
    BeschlRichtung_Sym = [];
    Phi1 = [];
    Phi2 = [];
    MaxPhi1 = [];
    MaxPhi2 = [];
    MinPhi1 = [];
    MinPhi2 = [];
    
    % Computing all the splines (one per axis) and backup rotation to have a z
    % reference rotation
    [achsstellung_A1,vilocity,acceleration,yerk,time1,splinePunkt_A1] = spline(base_points(:,1),timeintervals,false,schritte);
    [achsstellung_A2,~,~,~,~,~] = spline(base_points(:,2),timeintervals,false,schritte);
    [achsstellung_A3,~,~,~,~,~] = spline(base_points(:,3),timeintervals,false,schritte);
    [achsstellung_A4,~,~,~,~,~] = spline(base_points(:,4),timeintervals,false,schritte);
    [achsstellung_A5,~,~,~,~,~] = spline(base_points(:,5),timeintervals,false,schritte);
    [achsstellung_A6,~,~,~,~,~] = spline(base_points(:,6),timeintervals,false,schritte);
    [pos_initial, eulerZYX_initial] = vorwaertskinematik(base_points(1,:));
    
    %stimmt das so? Hier fehlt der Zeitliche Verlauf!!!!!!!!!!!!!!!
    MatrixKelleUngekippt = RotationUmZ(eulerZYX_initial(1));
    
    for i=1:length(achsstellung_A1)

        if ismember(i,splinePunkt_A1)
            if i == length(achsstellung_A1) || i == length(achsstellung_A1)-1
                p = [achsstellung_A1(i), achsstellung_A2(i), achsstellung_A3(i), achsstellung_A4(i), achsstellung_A5(i), achsstellung_A6(i)];
                v = [achsstellung_A1(i-1), achsstellung_A2(i-1), achsstellung_A3(i-1), achsstellung_A4(i-1), achsstellung_A5(i-1), achsstellung_A6(i-1)];
                [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p);
                [posv, eulerZYXV,eulerXYZV, y_directionv] = vorwaertskinematik(v);
                [acceleration, direction_acc,direction_acc_welt] = Beschleunigung(posv, pos, pos, (time1(i)-time1(i-1)),MatrixKelleUngekippt);
            elseif i == 1
                p = [achsstellung_A1(i), achsstellung_A2(i), achsstellung_A3(i), achsstellung_A4(i), achsstellung_A5(i), achsstellung_A6(i)];
                n = [achsstellung_A1(i+1), achsstellung_A2(i+1), achsstellung_A3(i+1), achsstellung_A4(i+1), achsstellung_A5(i+1), achsstellung_A6(i+1)];
                [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p); 
                [posn, kellen,eulerXYZN, y_directionn] = vorwaertskinematik(n); 
                [acceleration, direction_acc,direction_acc_welt] = Beschleunigung(pos, pos, posn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);     
            else
                p = [achsstellung_A1(i), achsstellung_A2(i), achsstellung_A3(i), achsstellung_A4(i), achsstellung_A5(i), achsstellung_A6(i)];
                n = [achsstellung_A1(i+1), achsstellung_A2(i+1), achsstellung_A3(i+1), achsstellung_A4(i+1), achsstellung_A5(i+1), achsstellung_A6(i+1)];
                nn = [achsstellung_A1(i+2), achsstellung_A2(i+2), achsstellung_A3(i+2), achsstellung_A4(i+2), achsstellung_A5(i+2), achsstellung_A6(i+2)];
                [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p); 
                [posn, kellen,eulerXYZN, y_directionn] = vorwaertskinematik(n); 
                [posnn, eulerZYXV,eulerXYZV, y_directionv] = vorwaertskinematik(nn); 
                [acceleration, direction_acc,direction_acc_welt] = Beschleunigung(pos, posn, posnn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);
            end
        
            % Setting better var names       
            direction_acc_welt = direction_acc_welt/norm(direction_acc_welt);
            start_z = eulerZYX_initial(1);        
        
            % Apply constraints only here:
            
            %constraints for table bounds
            %Falsche Werte da um 90° gedreht
%             c(end+1)= rx - max(simulation_data.rotationX); %X
%             c(end+1)= min(simulation_data.rotationX) - rx;
%             c(end+1)= ry - max(simulation_data.rotationY); %Y
%             c(end+1)= min(simulation_data.rotationY) - ry;
%             c(end+1) = abs(start_z - rz) - variance_z; %Z
%             c(end+1)= acceleration - (max(simulation_data.acceleration)/1000); %A
%             c(end+1)= (min(simulation_data.acceleration)/1000) - acceleration;


            phi1_max_tab = max(simulation_data.rotationX);
            phi2_max_tab = max(simulation_data.rotationY);

            phi1_min_tab = min(simulation_data.rotationX);
            phi2_min_tab = min(simulation_data.rotationY);
            
            %Daten aus der Simulationstabelle Auslesen für phimin 6 phimax
            Daten_Symulationstabelle = simulation_data(:,1:8);        
                         
            %Berechnung der Vorzugsrichtung aus der neigung gegeben durch eulerZYX   
            eulerZ_Sym = (eulerZYX(1,1)-90);
            if eulerZ_Sym < -180
                eulerZ_Sym = 180-abs(eulerZ_Sym + 180);
            elseif eulerZ_Sym > 180
                eulerZ_Sym = -180+abs(eulerZ_Sym - 180);
            end

            %Hin und Rücktransformation von TCP und Welt
            rot_TCP_zu_Welt =RotationUmZ(deg2rad(eulerZYX(1,1)))*RotationUmY(deg2rad(eulerZYX(1,2)))*RotationUmX(deg2rad(eulerZYX(1,3)));
            %rot_Welt_Zu_TCP =transpose(RotationUmX(deg2rad(eulerZYX(1,3))))*transpose(RotationUmY(deg2rad(eulerZYX(1,2))))*transpose(RotationUmZ(deg2rad(eulerZYX(1,1))));

            %Hin Und Rücktransformation von Sym und Welt
            %!!!!!!!!!!!!!!!!!!!!!Systeme um -90° zueiander gedreht!!!!!!!!!!!!!!!!!!                 
            rot_Sym_zu_Welt = RotationUmZ(deg2rad(eulerZ_Sym))*RotationUmY(deg2rad(0))*RotationUmX(deg2rad(0));
            rot_Welt_zu_Sym = transpose(RotationUmX(deg2rad(0)))*transpose(RotationUmY(deg2rad(0)))*transpose(RotationUmZ(deg2rad(eulerZ_Sym)));
            
            %Berechnung der Vorzugsrichtung in Weltkoordinaten
            vorzugsrichtung_TCP = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
            vorzugsrichtung = rot_TCP_zu_Welt*vorzugsrichtung_TCP; % die Transformation von Global in TCP-Koordinaten
            vorzugsrichtung_proj = [vorzugsrichtung(1),vorzugsrichtung(2),0]*1/sqrt(vorzugsrichtung(1)^2+vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor

            vorzugsrichtung_sym = RotationDegUmZ(90)*vorzugsrichtung;
            vorzugsrichtung_sym = transpose(vorzugsrichtung_sym);

            vorzugsrichtungPr_sym = RotationDegUmZ(90)*transpose(vorzugsrichtung_proj);
            vorzugsrichtungPr_sym = transpose(vorzugsrichtungPr_sym);
                            
            %Kegel außerhalb des Bereiches um Grad zu ereichen

            %berechnung der Winkel phi_1(Rotation in xy-ebene) phi2(winkel zur xy-ebene)
            phi_1 = CalculateAngleInXYPlane(transpose(vorzugsrichtung_proj),direction_acc_welt);
            phi_2 = 90 - (acos(dot([0 0 1],direction_acc_welt)/(norm(direction_acc_welt)))/pi * 180);

            %bei abweichungen von rx und ry die nicht in der Tabelle stehen wird unsinniges ausgegeben
            searchVector = [-eulerZYX(1,2),eulerZYX(1,3),0,vorzugsrichtungPr_sym(1,1),vorzugsrichtungPr_sym(1,2),vorzugsrichtungPr_sym(1,3),0,0];
            [k,dist] = dsearchn(table2array(Daten_Symulationstabelle),searchVector);   

            beschlRichtung_Sym = rot_Welt_zu_Sym*direction_acc_welt;
            beschlRichtung_Sym = beschlRichtung_Sym/sqrt(beschlRichtung_Sym(1,1)^2 + beschlRichtung_Sym(2,1)^2 + beschlRichtung_Sym(3,1)^2);
            beschlRichtung_Sym = transpose(beschlRichtung_Sym);
                        
% % % %             predictedAcceleration = regression.regressionKat2.predictFcn(table(-eulerZYX(1,2),eulerZYX(1,3),beschlRichtung_Sym(1,1),beschlRichtung_Sym(1,2),beschlRichtung_Sym(1,3),'VariableNames',{'rotx','roty','x_directionX_','x_directionY_','x_directionZ_'})); %rotx roty  x y z
% % % %             ceq(end+1) = ((predictedAcceleration/1000) - abs(acceleration))*10; %Variance ?
                      
            simu_CFD_phi1_k9 = table2array(simulation_data(k:(k+8),7));
            simu_CFD_phi2_k9 = table2array(simulation_data(k:(k+8),8));
        
            phi1_min = min(simu_CFD_phi1_k9);%-rz; %Um Welt in Sym zu drehen
            phi1_max = max(simu_CFD_phi1_k9);%-rz;
            
            phi2_min = min(simu_CFD_phi2_k9);
            phi2_max = max(simu_CFD_phi2_k9);
        
            %%%VisualizeAll(robot,p,direction_acc,true)

            %Eventuell durch Schwappen verarbeitet
            c(end+1) = phi_1 - phi1_max;
            c(end+1) = phi1_min - phi_1;
            c(end+1) = phi_2 - phi2_max;
            c(end+1) = phi2_min - phi_2;

               % Anhängen der Daten an die Arrays
            EulerZYX(end+1,:) = eulerZYX;
            Phi1(end+1,1) = phi_1;
            Phi2(end+1,1) = phi_2;
            MaxPhi1(end+1,1) = phi1_max_tab;
            MaxPhi2(end+1,1) = phi2_max_tab;
            MinPhi1(end+1,1) = phi1_min_tab;
            MinPhi2(end+1,1) = phi2_min_tab;
            BeschlRichtung_Sym(end +1,:) = beschlRichtung_Sym;
            directionX(end+1,1)=direction_acc_welt(1);
            directionY(end+1,1)=direction_acc_welt(2);
            directionZ(end+1,1)=direction_acc_welt(3);
            Acceleration(end+1,1)=acceleration;
                      
        end 

        %debug_output(i,rx, ry, rz, start_z, acceleration, direction, simulation_data, variance, variance_dir, variance_a);

        
    end
   
    %als ceq da beschleunigung erforderlich dh ist ein Sollwert bzw. = Value!!!
    for j=1:length(EulerZYX)        
        
        if Phi1(j,1) > phi1_min_tab && Phi1(j,1) < phi1_max_tab
            
            if Phi2(j,1) > phi2_min_tab && Phi2(j,1) < phi2_max_tab
                predictedAcceleration = regression.regressionKat4.predictFcn(table(-EulerZYX(j,2),EulerZYX(j,3),BeschlRichtung_Sym(j,1),BeschlRichtung_Sym(j,2),BeschlRichtung_Sym(j,3),'VariableNames',{'rotationX','rotationY','directionX','directionY','directionZ'})); %rotx roty  x y z 
                ceq(end+1) = ((predictedAcceleration/1000) - abs(Acceleration(j)))*2; %Variance ?

                fprintf('Schwappbedingung, fall im Korridor Erfüllt\n')
                fprintf(num2str(((predictedAcceleration/1000) - abs(Acceleration(j)))*2,j))
                fprintf('.\n')

                
            else          
                ceq(end+1) = (abs(Phi1(j))+abs(Phi2(j)))*5;
            end
        else
            ceq(end+1) = (abs(Phi1(j))+abs(Phi2(j)))*5;

        end               
% % % %         predictedAcceleration = regressionKat2.predictFcn(table(-EulerZYX(place1(i),2),EulerZYX(place1(i),3),BeschlRichtung_Sym(place1(i),1),BeschlRichtung_Sym(place1(i),2),BeschlRichtung_Sym(place1(i),3),'VariableNames',{'rotx','roty','x_directionX_','x_directionY_','x_directionZ_'})); %rotx roty  x y z
% % % %         ceq(end+1) = ((predictedAcceleration/1000) - abs(acceleration))*10; %Variance ?
      %%ceq(end+1) = ((predictedAcceleration(place1(i))/1000) - abs(Acceleration(place1(i))))*10; %Variance ?
% % %       c(end+1) = ((-predictedAcceleration(place1(i))/1000) + abs(Acceleration(place1(i))) - (variance_a/2))*100;
      %fprintf('Ist: %f Soll: %f\n',abs(Acceleration(place1(i))),abs(predictedAcceleration(place1(i))/1000))
    end
    
    % Sum-Constraint: amount of violations should be =0
    %ceq(end+1) = sum(abs(Acceleration(place1(i))) > abs(predictedAcceleration(place1(i))/1000)) - 0;
    %fprintf('Validation: %d / %d \n',sum(abs(Acceleration(place1(i))) > abs(predictedAcceleration(place1(i))/1000)), length(t1));
    %fprintf('Validation: %d / %d \n',sum(abs(abs(Acceleration(place1(i))) - abs(predictedAcceleration(place1(i))/1000))>variance_a), length(t1));
end


