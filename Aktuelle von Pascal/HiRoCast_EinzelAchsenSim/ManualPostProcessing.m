function [test] = ManualPostProcessing(splineDiscretization, visualizeTCPPath)
    % TODO: Visualisierung --> Alle Achsen mit Position, Geschwidigkeit,Beschleunigung, Ruck
    % TODO: TCP Position, Geschwindigkeit, Beschleunigung
    % TODO: Bei Über-/Unterschreitung Maxima / Minima anzeigen
    % TODO: Bearbeitung --> Position und Zeitintervalle verschieben, durch Texteingabe und Scrollen (Mausrad)
    % TODO: Bearbeitung --> Bei Veränderung alles aktualisieren
    % ACHTUNG: Start und Endposition sind fix
    % TODO: Richtige Visualisierung und Grenzen bei 'wayPoints
    % Visualization' --> Grenzen siehe: splineOptimization Zeilen 37 - 44

   example = matfile('SimResults.mat');
   optimized_translational_values_load = example.x;
   stop = 1;
   save('stop.mat','stop')

   Haendische_Nachbearbeitung("SimResults.mat");

   %Stoppen der Codeausführung wärend die App geöffnet ist.
   while stop == 1
        load('stop.mat','stop');
        pause(2);     
   end

% % %    editedVals = load('SimResults.mat');
% % % 
% % %    optimized_translational_values_load = editedVals.x;
% % % 
% % %    show_spline(optimized_translational_values_load, 'Axes Editor');
% % % 
% % %    tcppunkt = [];
% % %    eulZYX = [];
% % %    
% % %    
% % %    for p = 1:height(optimized_translational_values_load)
% % %         [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(optimized_translational_values_load(p,2:7));
% % %         wayPoints(:,p) = tcppunkt(:,1);
% % % % %         waypointsTCP = RotationDegUmZ(eulerZYX(1,1))*[]
% % %    end
% % %    wayPoints = wayPoints.';
% % % 
% % % %    location_Kat2D = spline(wayPoints(:,1),wayPoints(:,2));
% % %    [Xt,Xtd,Xtdd,Xtddd,Xtime,Xplace] = splineOptimal(wayPoints(:,1),optimized_translational_values_load(1:end-1,1),false);
% % %    [Yt,Ytd,Ytdd,Ytddd,Ytime,Yplace] = splineOptimal(wayPoints(:,2),optimized_translational_values_load(1:end-1,1),false);
% % % 
% % %    XYtd = [];
% % %    XYtdd = [];
% % %    XYtddd = [];
% % %    for i = 1:size(Xt,2)
% % %         XYtd(1:3,i) = RotationDegUmZ(eulZYX(i,1))*[Xtd(i);Ytd(i);0];
% % %         XYtdd(1:3,i) = RotationDegUmZ(eulZYX(i,1))*[Xtdd(i);Ytdd(i);0];
% % %         XYtddd(1:3,i) = RotationDegUmZ(eulZYX(i,1))*[Xtddd(i);Ytddd(i);0];
% % %    end
% % % 
% % %    abs_XYtd = sqrt(XYtd(1,:).^2+XYtd(2,:).^2);
% % %    abs_XYtdd = sqrt(XYtdd(1,:).^2+XYtdd(2,:).^2);
% % %    abs_XYtddd = sqrt(XYtddd(1,:).^2+XYtddd(1,:).^2);
% % %    
% % %    %Erstellen einer Ui Figure und setzen der Axes Eigenschaften
% % %    figure('units','normalized','outerposition',[0 0 1 1])
% % %    x = axes;
% % %    subplot(3,1,1)
% % %    plot(optimized_translational_values_load(:,1),abs_XYtd)
% % % 
% % %    subplot(3,1,2)
% % %    plot(optimized_translational_values_load(:,1),abs_XYtdd)
% % % 
% % %    subplot(3,1,3)
% % %    plot(optimized_translational_values_load(:,1),abs_XYtddd)
% % %    
% % %    plotWayPoints = [optimized_translational_values_load(:, 1) wayPoints wayPoints];
% % % 
% % %    show_spline(plotWayPoints, 'wayPoints Visualization');
end

