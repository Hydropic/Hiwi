function [c,ceq] = completeValidation(timeintervals, base_points)
    fprintf('Validation spline + base\n');
    %Validates of a path sloshes
    %   Input: Basepoints, Computes spline, iterates over all points (extra
    %   cases first and last point), sets constraints on forward kinematics
    %   Outputs: Constraint Arrays: sum of validation and special values at
    %   basepoints
    
    c = [];
    ceq = [];
    
    %schritte = 1000;
    schritte = 50; %voher 50

    %Um plotten des Roboters zu ermöglichen
    KSetUp
    
    % Setting variance for acceleration
    variance_a = 0.5; 
    variance_z = 5;%in Deg
    
    % Loading simulation data (column names differ slightly from regression Model)
    simulation_data = readtable('simulationData.txt');
    simulation_data.Properties.VariableNames = {'rotationX','rotationY','rotationZ','directionX','directionY','directionZ','horizChange','vertChange','acceleration'};
    
    % Loading the trained Regression Model
    load("regressionKat2.mat")
    
    orientierung_mesh = simulation_data(:,1:3);
    
    % Creating empty table-arrays
    rotationX = [];
    rotationY = [];
    directionX = [];
    directionY = [];
    directionZ = [];
    Acceleration = [];
    
    % Computing all the splines (one per axis) and backup rotation to have a z
    % reference rotation
    [t1,td1,tdd1,tddd1,time1,place1] = spline(base_points(:,1),timeintervals,false,schritte);
    [t2,td2,tdd2,tddd2,time2,place2] = spline(base_points(:,2),timeintervals,false,schritte);
    [t3,td3,tdd3,tddd3,time3,place3] = spline(base_points(:,3),timeintervals,false,schritte);
    [t4,td4,tdd4,tddd4,time4,place4] = spline(base_points(:,4),timeintervals,false,schritte);
    [t5,td5,tdd5,tddd5,time5,place5] = spline(base_points(:,5),timeintervals,false,schritte);
    [t6,td6,tdd6,tddd6,time6,place6] = spline(base_points(:,6),timeintervals,false,schritte);
    [pos_backup, eulerZYX_backup] = vorwaertskinematik(base_points(1,:));
    
    %stimmt das so?
    MatrixKelleUngekippt = RotationUmZ(eulerZYX_backup(1));
    
    for i=1:length(t1)
        if i == length(t1) | i == length(t1)-1
            p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
            v = [t1(i-1), t2(i-1), t3(i-1), t4(i-1), t5(i-1), t6(i-1)];
            [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p);
            [posv, eulerZYXV,eulerXYZV, y_directionn] = vorwaertskinematik(v);
            [acceleration, direction_acc] = Beschleunigung(posv, pos, pos, (time1(i)-time1(i-1)),MatrixKelleUngekippt);
        elseif i == 1
            p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
            n = [t1(i+1), t2(i+1), t3(i+1), t4(i+1), t5(i+1), t6(i+1)];
            [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p); 
            [posn, kellen,eulerXYZN, y_directionn] = vorwaertskinematik(n); 
            [acceleration, direction_acc] = Beschleunigung(pos, pos, posn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);     
        else
            p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
            n = [t1(i+1), t2(i+1), t3(i+1), t4(i+1), t5(i+1), t6(i+1)];
            nn = [t1(i+2), t2(i+2), t3(i+2), t4(i+2), t5(i+2), t6(i+2)];
            [pos, eulerZYX,eulerXYZ, y_direction] = vorwaertskinematik(p); 
            [posn, kellen,eulerXYZN, y_directionn] = vorwaertskinematik(n); 
            [posnn, eulerZYXV,eulerXYZV, y_directionv] = vorwaertskinematik(nn); 
            [acceleration, direction_acc] = Beschleunigung(pos, posn, posnn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);
        end
    
        % Setting better var names
        rx = eulerZYX(3);
        ry = eulerZYX(2); 
        rz = eulerZYX(1);        
        direction_acc = direction_acc/norm(direction_acc);
        start_z = eulerZYX_backup(1);
    
        %Winkel umrechnen, da Eulerwinkel nicht über 180 Grad gehen, sondern
        %negative Winkel angeben, was beim Vergleich sonst Probleme macht 
        %Fall 1: my_z ist positiv und rz ist negativ 
        if start_z > 180-variance_z

            if rz < 0 
                rz = abs(180+rz)+180;            
            end
        end 
        %Fall 2: my_z ist negativ und rz ist positiv 
        if start_z < -180+variance_z
 
            if rz > 0
                start_z = abs(180+start_z)+180;
            end
        end 
    
        % Apply constraints only here:
        if ismember(i,place1)
            %constraints for table bounds

            %Eventuell weglassen, indirekt in phi1 und phi2


            %Falsche Werte da um 90° gedreht
            c(end+1)= rx - max(simulation_data.rotationX); %X
            c(end+1)= min(simulation_data.rotationX) - rx;
            c(end+1)= ry - max(simulation_data.rotationY); %Y
            c(end+1)= min(simulation_data.rotationY) - ry;
    % % % %         c(end+1) = abs(start_z - rz) - variance_z; %Z
    % % % %         c(end+1)= acceleration - (max(simulation_data.acceleration)/1000); %A
    % % % %         c(end+1)= (min(simulation_data.acceleration)/1000) - acceleration;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Winkel Bedingung%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %Daten aus der Simulationstabelle Auslesen für phimin 6 phimax
            Daten_Symulationstabelle = simulation_data(:,1:8);
        
            kellen_orient_Acc_neu = [rx ry rz direction_acc(1) direction_acc(2) direction_acc(3) acceleration]; % Rx Ry Rz nx ny nz A              
            beschlRichtung = kellen_orient_Acc_neu(:,4:6);%Weltkoord!!
                         
            %Berechnung der Vorzugsrichtung aus der neigung gegeben durch eulerZYX   
            eulerZ_Sym = (eulerZYX(1,1)-90);
            if eulerZ_Sym < -180
                eulerZ_Sym = 180-abs(eulerZ_Sym + 180);
            elseif eulerZ_Sym > 180
                eulerZ_Sym = -180+abs(eulerZ_Sym - 180);
            end

            %Hin und Rücktransformation von TCP und Welt
            rot_TCP_zu_Welt =RotationUmZ(deg2rad(eulerZYX(1,1)))*RotationUmY(deg2rad(eulerZYX(1,2)))*RotationUmX(deg2rad(eulerZYX(1,3)));
            rot_Welt_Zu_TCP =transpose(RotationUmX(deg2rad(eulerZYX(1,3))))*transpose(RotationUmY(deg2rad(eulerZYX(1,2))))*transpose(RotationUmZ(deg2rad(eulerZYX(1,1))));

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
            phi_1 = CalculateAngleInXYPlane(vorzugsrichtung_proj,beschlRichtung);
            phi_2 = 90 - (acos(dot([0 0 1],beschlRichtung)/(norm(beschlRichtung)))/pi * 180);

            %bei abweichungen von rx und ry die nicht in der Tabelle stehen wird unsinniges ausgegeben
            searchVector = [-eulerZYX(1,2),eulerZYX(1,3),0,vorzugsrichtungPr_sym(1,1),vorzugsrichtungPr_sym(1,2),vorzugsrichtungPr_sym(1,3),0,0];
            [k,dist] = dsearchn(table2array(Daten_Symulationstabelle),searchVector);   

            beschlRichtung_Sym = rot_Welt_zu_Sym*transpose(beschlRichtung);
            beschlRichtung_Sym = beschlRichtung_Sym/sqrt(beschlRichtung_Sym(1,1)^2 + beschlRichtung_Sym(2,1)^2 + beschlRichtung_Sym(3,1)^2);
            beschlRichtung_Sym = transpose(beschlRichtung_Sym);
            
            
% % % % % %             predictedAcceleration = regressionKat2.predictFcn(table(-eulerZYX(1,2),eulerZYX(1,3),beschlRichtung_Sym(1,1),beschlRichtung_Sym(1,2),beschlRichtung_Sym(1,3),'VariableNames',{'rotx','roty','x_directionX_','x_directionY_','x_directionZ_'})); %rotx roty  x y z
% % % % % %             ceq(end+1) = ((predictedAcceleration/1000) - abs(acceleration))*10; %Variance ?
                      
            simu_CFD_phi1_k9 = table2array(simulation_data(k:(k+8),7));%winkel Horizontal müsste Rz entsprechen
            simu_CFD_phi2_k9 = table2array(simulation_data(k:(k+8),8));%winkel vertikal müsste Rz*Ry entsprechen
        
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


            
        end
                   
        % Anhängen der Daten an die Arrays
        rotationX(end+1,1)=rx;
        rotationY(end+1,1)=ry;
        directionX(end+1,1)=direction_acc(1);
        directionY(end+1,1)=direction_acc(2);
        directionZ(end+1,1)=direction_acc(3);
        Acceleration(end+1,1)=acceleration;
    
        %debug_output(i,rx, ry, rz, start_z, acceleration, direction, simulation_data, variance, variance_dir, variance_a);
    end
    
            %TODO Polarkoordinaten/Vorzugsrichtungsvarianz
        %end
    
    %Predicting the acceleration for every state
%%%     predictedAcceleration = regressionkat.predictFcn(table(directionX,directionY,directionZ,rotationX,rotationY));
    
    % Setting constraints fot he basepoints
    

    %als ceq da beschleunigung erforderlich
    for i=1:size(base_points,1) 
      %%%%%ceq(end+1) = ((predictedAcceleration(place1(i))/1000) - abs(Acceleration(place1(i))))*10; %Variance ?
      %%c(end+1) = ((-predictedAcceleration(place1(i))/1000) + abs(Acceleration(place1(i))) - (variance_a/2))*100;
       %%%%fprintf('Ist: %f Soll: %f\n',abs(Acceleration(place1(i))),abs(predictedAcceleration(place1(i))/1000))
    end
    
    % Sum-Constraint: amount of violations should be =0
    %ceq(end+1) = sum(abs(Acceleration(place1(i))) > abs(predictedAcceleration(place1(i))/1000)) - 0;
    %fprintf('Validation: %d / %d \n',sum(abs(Acceleration(place1(i))) > abs(predictedAcceleration(place1(i))/1000)), length(t1));
    %fprintf('Validation: %d / %d \n',sum(abs(abs(Acceleration(place1(i))) - abs(predictedAcceleration(place1(i))/1000))>variance_a), length(t1));
end
    
function debug_output(i,rx, ry, rz, start_z, acceleration, direction, simulation_data, variance, variance_dir, variance_a)
    % Debug output    
    fprintf('Iter. %d X:%f Y:%f Z:%f [Z:%f] A:%f dx: %f dy: %f dz: %f\n',i,rx,ry,rz,start_z,acceleration,direction(1),direction(2),direction(3));
    
    % Printing entries with valid rotation
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance);
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('Rotation: [%s] \n',debug_string);
    end
    
    % Printing entries with valid direction(acceleration) %
    % acceleration
    debug_index = (abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('Direction: [%s] \n',debug_string);
    end
    
    % Printing entries with valid rotation & direction(acceleration)
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance & ...
        abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir &...
        abs(direction(3) - simulation_data.directionZ) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('#Rotation+Direction: [%s] \n',debug_string);
    end
    
    % Printing entries with valid rotation & direction(acceleration) %
    % acceleration
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance & ...
        abs(acceleration - simulation_data.acceleration) <= variance_a & ...
        abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir &...
        abs(direction(3) - simulation_data.directionZ) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('##Alles [%s] \n',debug_string);
    end
end