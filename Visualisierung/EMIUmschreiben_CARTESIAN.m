close all;
clear all;

%% Eingabe
beschlIntervallGroesse = 0.025
AnzahlIntervall = 30
beschleunigungswert_X = 0.025 % m/s²
beschleunigungswert_Y = 0 % m/s²
beschleunigungswert_Z = 0 % m/s²
ZeitEnde = 14.2 % Sekunden
reverse = 0;

EMIFile = "Data/Emily_Complex_C_gekuerzt.EMI";

NameFile = erase(EMIFile,"Data/");
NameFile = erase(NameFile,".EMI");


%% Laden der Datei
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

[aa, bb] = size(timeData);

for steps = 0:aa-1
    s_x = [];
    s_y = [];
    s_z = [];

            %% Zeit Einstellen
             if steps == 0
                 zeitstep(1) = 0
             else
                 zeitstep(end+1) = ZeitEnde*(steps/(aa-1))
             end
            
end
    %% Umorientierung
    orientation_X = 0;
    orientation_Y = 0;
    orientation_Z = 0;
    
    %% Speichern als EMI TCP
    Position_xyz = [s_x.', s_y.', s_z.'];

    optimized_translational_values_sameDistances(:, 1) = zeitstep';
    optimized_translational_values_sameDistances(:, 2) = first_Axis;
    optimized_translational_values_sameDistances(:, 3) = second_Axis;
    optimized_translational_values_sameDistances(:, 4) = third_Axis;
    optimized_translational_values_sameDistances(:, 5) = +0 + orientation_X;
    optimized_translational_values_sameDistances(:, 6) = -90 + orientation_Y;
    optimized_translational_values_sameDistances(:, 7) = +0 + orientation_Z;
    
    example = readcell('Emily1_blanko.txt');
    example(end+1,1) = {'[HEADER]'};
    example(end+1,1) = {'  GEAR_NOMINAL_VEL = 1.000000'};
    example(end+1,1) = {'  SAMPLING_MODE = CARTESIAN'};
    example(end+1,1) = {'  CRC = 4230818067'};
    example(end+1,1) = {'[RECORDS]'};
    
    for i = 1:height(optimized_translational_values_sameDistances)
    
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
        txt = txt + ' +18 +10';
        example(end+1,1) = cellstr(txt(:));
    end
    example(end+1,1) = {'[END]'};
    
    mask = cellfun(@(x) any(isa(x,'missing')), example); % using isa instead of ismissing allows white space through
    example(mask) = {['']}

    nameFile = [num2str(NameFile, 2) '_Zeit_' num2str(ZeitEnde) '.txt']
    
    writecell(example, nameFile, 'Delimiter',' ')
