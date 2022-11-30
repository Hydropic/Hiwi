function [discretizedsSplinePath] = show6DoF(numSamples)
    delimiter='(';
    opts=delimitedTextImportOptions('Delimiter',delimiter);

    A=readmatrix("C:\Users\penczek\Desktop\Emily_optimiert\OpenFoam - Sim-Ilyas\6DoF.dat",opts);
    A(1,:) = [];
    A(end,:) = [];
    for re = 1:length(A)
        f = A(re,4)
        f = replace(f,'0	','')
        f = replace(f,'	','')
        f = replace(f,'(','')
        f = replace(f,')','')
        A(re,4) = cell(f)
    end
    
    B = str2double(A)

    for r = 1:length(B)
        if r == 1
            splinePath(r, 1) = B(r, 2);
        else
            eins = str2double(B(r, 2)) - str2double(B(r-1, 2));
            splinePath(r, 1) = B(r, 2) - B(r-1, 2);
        end
        
        splinePath(r, 2) = B(r, 4);
        splinePath(r, 3) = B(r, 4);
        splinePath(r, 4) = B(r, 4);
        splinePath(r, 5) = B(r, 4);
        splinePath(r, 6) = B(r, 4);
        splinePath(r, 7) = B(r, 4);
    end
    splinePath(1, :) = [];
    number = 200 / numSamples
    discretizedsSplinePath = splinePath(1:number:end,:);
    discretizedsSplinePath(:,3) = [];
    discretizedsSplinePath(:,3) = [];
    discretizedsSplinePath(:,3) = [];
    discretizedsSplinePath(:,3) = [];
    discretizedsSplinePath(:,3) = [];

    value100 = discretizedsSplinePath(end, 2);

    for l = 1:length(discretizedsSplinePath)
        discretizedsSplinePath(l,2) = discretizedsSplinePath(l,2)/value100
    end

    % ab Zeitschritt 0.741 (10) bis 1.00 (13) konstant Steigung

    % Eingangswert für den Fixpunkt implementieren
    % Entscheiden, ob Punkt zwischen Start und End oder drunter oder drüber

    discretizedsSplinePath(:,1) = 0.1;
    discretizedsSplinePathPlot(:,1) = discretizedsSplinePath(:,1)
    discretizedsSplinePathPlot(:,2) = discretizedsSplinePath(:,2)
    discretizedsSplinePathPlot(:,3) = discretizedsSplinePath(:,2)
    discretizedsSplinePathPlot(:,4) = discretizedsSplinePath(:,2)
    discretizedsSplinePathPlot(:,5) = discretizedsSplinePath(:,2)
    discretizedsSplinePathPlot(:,6) = discretizedsSplinePath(:,2)
    discretizedsSplinePathPlot(:,7) = discretizedsSplinePath(:,2)

    factor = 0.00
    for as = 1:length(discretizedsSplinePathPlot)
        pos = ((length(discretizedsSplinePath)-(as-1))/length(discretizedsSplinePath))
        %sddsd = as*discretizedsSplinePath(as,2)*factor
        discretizedsSplinePathPlot(as,1) = discretizedsSplinePath(as,1) + pos*factor % - sddsd
    end

    for as = length(discretizedsSplinePathPlot):-1:1
        pos = as/length(discretizedsSplinePath)
        %sddsd = as*discretizedsSplinePath(as,2)*factor
        discretizedsSplinePathPlot(as,1) = discretizedsSplinePathPlot(as,1) - pos*factor % - sddsd
    end

%     show_spline(discretizedsSplinePathPlot, 'y (um Achse 6), x (um Achse 5)');
end

