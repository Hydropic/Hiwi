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
schritte = 50;

% Setting variance for acceleration
variance_a = 0.7; 
variance_z = 10;

% Loading simulation data (column names differ slightly from regression Model)
simulation_data = readtable('simulationData.txt');
simulation_data.Properties.VariableNames = {'rotationX','rotationY','rotationZ','directionX','directionY','directionZ','horizChange','vertChange','acceleration'};

% Loading the trained Regression Model
load("regressionKat.mat")





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
[pos_backup, kelle_backup] = vorwaertskinematik(base_points(1,:));

MatrixKelleUngekippt = RotationUmZ(kelle_backup(1));

for i=1:length(t1)
    if i == length(t1) | i == length(t1)-1
        p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
        v = [t1(i-1), t2(i-1), t3(i-1), t4(i-1), t5(i-1), t6(i-1)];
        [pos, kelle, y_direction] = vorwaertskinematik(p);
        [posv, kellev, y_directionn] = vorwaertskinematik(v);
        [acceleration, direction] = Beschleunigung(posv, pos, pos, (time1(i)-time1(i-1)),MatrixKelleUngekippt);
    elseif i == 1
        p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
        n = [t1(i+1), t2(i+1), t3(i+1), t4(i+1), t5(i+1), t6(i+1)];
        [pos, kelle, y_direction] = vorwaertskinematik(p); 
        [posn, kellen, y_directionn] = vorwaertskinematik(n); 
        [acceleration, direction] = Beschleunigung(pos, pos, posn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);     
    else
        p = [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
        n = [t1(i+1), t2(i+1), t3(i+1), t4(i+1), t5(i+1), t6(i+1)];
        nn = [t1(i+2), t2(i+2), t3(i+2), t4(i+2), t5(i+2), t6(i+2)];
        [pos, kelle, y_direction] = vorwaertskinematik(p); 
        [posn, kellen, y_directionn] = vorwaertskinematik(n); 
        [posnn, kellev, y_directionv] = vorwaertskinematik(nn); 
        [acceleration, direction] = Beschleunigung(pos, posn, posnn, (time1(i+1)-time1(i)),MatrixKelleUngekippt);
    end

    % Setting better var names
    rx = kelle(3);
    ry = kelle(2); 
    rz = kelle(1); 
    direction = direction/norm(direction);
    start_z = kelle_backup(1);

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
        c(end+1)= rx - max(simulation_data.rotationX); %X
        c(end+1)= min(simulation_data.rotationX) - rx;
        c(end+1)= ry - max(simulation_data.rotationY); %Y
        c(end+1)= min(simulation_data.rotationY) - ry;
        c(end+1) = abs(start_z - rz) - variance_z; %Z
        c(end+1)= acceleration - (max(simulation_data.acceleration)/1000); %A
        c(end+1)= (min(simulation_data.acceleration)/1000) - acceleration;
    end
        

        

    % Anhängen der Daten an die Arrays
    rotationX(end+1,1)=rx;
    rotationY(end+1,1)=ry;
    directionX(end+1,1)=direction(1);
    directionY(end+1,1)=direction(2);
    directionZ(end+1,1)=direction(3);
    Acceleration(end+1,1)=acceleration;

    %debug_output(i,rx, ry, rz, start_z, acceleration, direction, simulation_data, variance, variance_dir, variance_a);
end

%%% Winkel Bedingung
       orientierung_mesh = simulation_data(:,1:3);
    
       kelle_orientierung_beschleunigung_neu = [rx ry rz direction(1) direction(2) direction(3) acceleration]; % Rx Ry Rz nx ny nz A
       kelle_orientierung = kelle_orientierung_beschleunigung_neu(:,1:3);
       kelle_richtung_neu = kelle_orientierung_beschleunigung_neu(:,4:6);

       [k,dist] = dsearchn(table2array(orientierung_mesh),kelle_orientierung);
      
       simu_CFD_id_k = simulation_data(k,:);

       vorzugsBewegungsrichtung = simu_CFD_id_k(:,4:6);

       phi_1 = CalculateAngleInXYPlane(table2array(vorzugsBewegungsrichtung),kelle_richtung_neu);

       phi_2 = 90 - (acos(dot([0 0 1],kelle_richtung_neu)/(norm(kelle_richtung_neu)))/pi * 180);

       simu_CFD_phi1_k9 = table2array(simulation_data(k:(k+8),7));
       simu_CFD_phi2_k9 = table2array(simulation_data(k:(k+8),8));

       phi1_min = min(simu_CFD_phi1_k9);
       phi1_max = max(simu_CFD_phi1_k9);


       phi2_min = min(simu_CFD_phi2_k9);
       phi2_max = max(simu_CFD_phi2_k9);

       c(end+1) = phi_1 - phi1_max;
       c(end+1) = phi1_min - phi_1;
       c(end+1) = phi_2 - phi2_max;
       c(end+1) = phi2_min - phi_2;
       
       %%%% Winkelbedingung End
        %TODO Polarkoordinaten/Vorzugsrichtungsvarianz
    %end

%Predicting the acceleration for every state
%predictedAcceleration= regressionModel.predictFcn(table(RotationX,RotationY,VectorX,VectorY,VectorZ));
predictedAcceleration = regressionkat.predictFcn(table(directionX,directionY,directionZ,rotationX,rotationY));
% EDITED !!!!!fprintf('Länge predictions: %d\n',length(predictedAcceleration));
%TODO acceleration is als Betrag oder??

% Setting constraints fot he basepoints
for i=1:size(base_points,1) 
    c(end+1) = (predictedAcceleration(place1(i))/1000) - abs(Acceleration(place1(i))) - (variance_a/2);
    c(end+1) = (-predictedAcceleration(place1(i))/1000) + abs(Acceleration(place1(i))) - (variance_a/2);
   % fprintf('Ist: %f Soll: %f\n',abs(Acceleration(place1(i))),abs(predictedAcceleration(place1(i))/1000))
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