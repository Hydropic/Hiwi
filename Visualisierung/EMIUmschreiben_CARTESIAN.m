close all;
clear all;

%% Eingabe
EMIFile = "input/InputSOBGB_opti_DG_Emily_TCP.txt";
wenigerFeinDiskretisieren = 1;
jedeXbelassen = 4;


beschlIntervallGroesse = 0.025
AnzahlIntervall = 30
beschleunigungswert_X = 0.025 % m/s²
beschleunigungswert_Y = 0 % m/s²
beschleunigungswert_Z = 0 % m/s²
ZeitEnde = 2.4143 % Sekunden
reverse = 0;

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

timeData_kurz = [];
first_Axis_kurz = [];
second_Axis_kurz = [];
third_Axis_kurz = [];
fourth_Axis_kurz = [];
fifth_Axis_kurz = [];
sixth_Axis_kurz = [];

%% Anzahl an Intervallen Kürzen
if wenigerFeinDiskretisieren
    kuerzereListe = [];
    [aa, bb] = size(timeData);

    for z = 1:jedeXbelassen:aa
        timeData_kurz(end+1) = timeData(z)
        first_Axis_kurz(end+1,:) = first_Axis(z)
        second_Axis_kurz(end+1,:) = second_Axis(z)
        third_Axis_kurz(end+1,:) = third_Axis(z)
        fourth_Axis_kurz(end+1,:) = fourth_Axis(z)
        fifth_Axis_kurz(end+1,:) = fifth_Axis(z)
        sixth_Axis_kurz(end+1,:) = sixth_Axis(z)
    end

    if aa > z
        timeData_kurz(end+1) = timeData(aa)
        first_Axis_kurz(end+1) = first_Axis(aa)
        second_Axis_kurz(end+1) = second_Axis(aa)
        third_Axis_kurz(end+1) = third_Axis(aa)
        fourth_Axis_kurz(end+1) = fourth_Axis(aa)
        fifth_Axis_kurz(end+1) = fifth_Axis(aa)
        sixth_Axis_kurz(end+1) = sixth_Axis(aa)
    end

    [aa, bb] = size(timeData_kurz);


    summeAlleZeilen = sum(timeData,1);
    zeiintervalle = summeAlleZeilen(:,1)/(aa-1);

    for z = 1:aa
        timeData_kurz(z,1) = zeiintervalle
    end
    

    timeData = timeData_kurz;
    first_Axis = first_Axis_kurz;
    second_Axis = second_Axis_kurz;
    third_Axis = third_Axis_kurz;
    fourth_Axis = fourth_Axis_kurz;
    fifth_Axis = fifth_Axis_kurz;
    sixth_Axis = sixth_Axis_kurz;


    [aa, bb] = size(timeData);

    zeitstep = [];

    for steps = 0:bb-1
        s_x = [];
        s_y = [];
        s_z = [];
    
                %% Zeit Einstellen
                 if steps == 0
                     zeitstep(1) = 0
                 else
                     zeitstep(end+1) = ZeitEnde*(steps/(bb-1))
                 end
                
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
    
    example = readcell('input/Emily1_blanko.txt');
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
