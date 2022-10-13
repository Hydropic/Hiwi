close all;
clear all;

%% Eingabe
beschlIntervallGroesse = 0.2
AnzahlIntervall = 30
beschleunigungswert_X = 0.8 % m/s²
beschleunigungswert_Y = 0 % m/s²
beschleunigungswert_Z = 0 % m/s²
Zeitintervall = 1.6 % Sekunden
Zeitschritte = 0.2 % Sekunden

for steps = 1:AnzahlIntervall
    if beschleunigungswert_X == 0
    else
        beschleunigungswert_X = beschleunigungswert_X + beschlIntervallGroesse
    end
    if beschleunigungswert_Y == 0
    else
        beschleunigungswert_Y = beschleunigungswert_Y + beschlIntervallGroesse
    end
    if beschleunigungswert_Z == 0
    else
        beschleunigungswert_Z = beschleunigungswert_Z + beschlIntervallGroesse
    end    
    Anzahl = (Zeitintervall/Zeitschritte)+1;
    
    %% Erzeugen einer Bahn und Ableitung
    ZeitAchse = linspace(0, Zeitintervall, Anzahl);
    s_x = [];
    s_y = [];
    s_z = [];
    
    for t = 1:length(ZeitAchse)
        s_x(end+1) = 0.5*beschleunigungswert_X*ZeitAchse(t)^2
        s_y(end+1) = 0.5*beschleunigungswert_Y*ZeitAchse(t)^2
        s_z(end+1) = 0.5*beschleunigungswert_Z*ZeitAchse(t)^2
    end
    
    test = 0;
    
    %% Umorientierung
    orientation_X = 0;
    orientation_Y = 30;
    orientation_Z = 0;
    
    %% Speichern als EMI TCP
    Position_xyz = [s_x.', s_y.', s_z.'];
    optimized_translational_values_sameDistances(:, 1) = transpose(ZeitAchse);
    optimized_translational_values_sameDistances(:, 2:4) = 1000*Position_xyz;
    optimized_translational_values_sameDistances(:, 5) = -90 + orientation_X;
    optimized_translational_values_sameDistances(:, 6) = 0 + orientation_Y;
    optimized_translational_values_sameDistances(:, 7) = +90 + orientation_Z;
    
    optimized_translational_values_sameDistances(1, 5) = -90;
    optimized_translational_values_sameDistances(1, 6) = 0;
    optimized_translational_values_sameDistances(1, 7) = +90;
    
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
        txt = txt + ' +22 +50';
        example(end+1,1) = cellstr(txt(:));
    end
    example(end+1,1) = {'[END]'};
    
    mask = cellfun(@(x) any(isa(x,'missing')), example); % using isa instead of ismissing allows white space through
    example(mask) = {['']}

    nameFile = ['LarsExampleDataCopy_x-' num2str(beschleunigungswert_X, 2) '_y-' num2str(beschleunigungswert_Y) '_z-' num2str(beschleunigungswert_Z) '.txt']
    
    writecell(example, nameFile, 'Delimiter',' ')
end