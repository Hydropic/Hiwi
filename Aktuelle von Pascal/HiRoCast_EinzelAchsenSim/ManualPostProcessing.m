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
    show_spline(optimized_translational_values_load, 'Axes Editor');

   for p = 1:height(optimized_translational_values_load)
        [tcppunkt, eulZYX, eulXYZ, RichtungInTCP, winkelmatrix] = vorwaertskinematik(optimized_translational_values_load(p,2:7))
        wayPoints(:,p) = tcppunkt(:,1)
   end
   wayPoints = wayPoints.'

   plotWayPoints = [optimized_translational_values_load(:, 1) wayPoints wayPoints];

   show_spline(plotWayPoints, 'wayPoints Visualization');
end

