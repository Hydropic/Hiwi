function [test] = ManualPostProcessing(splineDiscretization, visualizeTCPPath,max_values,min_values,jerkBoundaries)
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

   x = optimized_translational_values_load;

   x2 = [];
   iXteLoeschen = 2;
   for kkk = 1:iXteLoeschen:length(x)
       zeitintervall = x(1,1)*iXteLoeschen
       x2(end+1,:) = x(kkk,:);
       x2(:,1) = zeitintervall; 
   end

   x = x2;
   optimized_translational_values_load = x;
   show_spline(optimized_translational_values_load, 'Axes Editor');
   save('SimResults_klein.mat','x','-v7.3');

   stop = 1;
   save('stop.mat','stop')

   Haendische_Nachbearbeitung("SimResults_klein.mat");

   %Stoppen der Codeausführung wärend die App geöffnet ist.
   while stop == 1
        load('stop.mat','stop');
        pause(2);     
   end

   editedVals = load('SimResults.mat');

   optimized_translational_values_load = editedVals.x;

   show_spline(optimized_translational_values_load, 'Axes Editor');

   tcppunkt = [];
   eulZYX = [];
   
   for p = 1:height(optimized_translational_values_load)
        [tcppunkt, eul, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(optimized_translational_values_load(p,2:7));
        wayPoints(:,p) = tcppunkt(:,1);
        eulZYX(1:3,p) = eul;
   end
   wayPoints = wayPoints.';

   [Xt_W,Xtd_W,Xtdd_W,Xtddd_W,Xtime_W,Xplace_W] = splineOptimal(wayPoints(:,1),optimized_translational_values_load(1:end-1,1),false);
   [Yt_W,Ytd_W,Ytdd_W,Ytddd_W,Ytime_W,Yplace_W] = splineOptimal(wayPoints(:,2),optimized_translational_values_load(1:end-1,1),false);
   [eulZYXt_W,~,~,~,~,~] = splineOptimal(eulZYX(1,:),optimized_translational_values_load(1:end-1,1),false);
   eulTransposed = transpose(eulZYXt_W);

   XYtd_TCP = [];
   XYtdd_TCP = [];
   XYtddd_TCP = [];
   for i = 1:size(Xt_W,2)
        XYtd_TCP(i,1:3) = transpose(RotationDegUmZ(-eulZYXt_W(1,i))*[Xtd_W(1,i);Ytd_W(1,i);0]);
        XYtdd_TCP(i,1:3) = transpose(RotationDegUmZ(-eulZYXt_W(1,i))*[Xtdd_W(1,i);Ytdd_W(1,i);0]);
        XYtddd_TCP(i,1:3) = transpose(RotationDegUmZ(-eulZYXt_W(1,i))*[Xtddd_W(1,i);Ytddd_W(1,i);0]);
   end

   abs_XYtd = sqrt(XYtd_TCP(:,1).^2+XYtd_TCP(:,2).^2);
   abs_XYtdd = sqrt(XYtdd_TCP(:,1).^2+XYtdd_TCP(:,2).^2);
   abs_XYtddd = sqrt(XYtddd_TCP(:,1).^2+XYtddd_TCP(:,2).^2);
   
   %Erstellen einer Ui Figure und setzen der Axes Eigenschaften
   figure('units','normalized','outerposition',[0 0 1 1])
   x = axes;
   subplot(3,1,1)
   hold 'on' 
   plot(Xtime_W,abs_XYtd,'--b','LineWidth',1)
   plot(Xtime_W,XYtd_TCP(:,1),'-g','LineWidth',1)
   plot(Xtime_W,XYtd_TCP(:,2),'-r','LineWidth',1.5)
   yline(max_values(2,2),'--','Max','LineWidth',0.75,'LabelVerticalAlignment','bottom','LabelHorizontalAlignment','left')
   yline(min_values(2,2),'--','Min','LineWidth',0.75,'LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
   xlabel('t')
   ylabel('Velocities')
   legend('Absolut','X','Y') 

   subplot(3,1,2)
   hold 'on' 
   plot(Xtime_W,abs_XYtdd,'--b','LineWidth',1)   
   plot(Xtime_W,XYtdd_TCP(:,1),'-g','LineWidth',1)
   plot(Xtime_W,XYtdd_TCP(:,2),'-r','LineWidth',1.5)
   yline(max_values(2,4),'--','Max','LineWidth',0.75,'LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
   yline(min_values(2,4),'--','Min','LineWidth',0.75,'LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
   yline(2.5,'--','Max Y','LineWidth',0.75,'LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
   yline(-2.5,'--','Min Y','LineWidth',0.75,'LabelVerticalAlignment','top','LabelHorizontalAlignment','left')
   xlabel('t')
   ylabel('Acceleration')
   legend('Absolut','X','Y')

   subplot(3,1,3)
   hold 'on' 
   plot(Xtime_W,abs_XYtddd,'--b','LineWidth',1)
   plot(Xtime_W,XYtddd_TCP(:,1),'-g','LineWidth',1)
   plot(Xtime_W,XYtddd_TCP(:,2),'-r','LineWidth',1.5)
   yline(jerkBoundaries,'--','LineWidth',0.75)
   yline(-jerkBoundaries,'--','LineWidth',0.75)
   ylabel('Yerk')
   legend('Absolut','X','Y')
   
   plotWayPoints = [optimized_translational_values_load(:, 1) wayPoints wayPoints];

   show_spline(plotWayPoints, 'wayPoints Visualization');
end

