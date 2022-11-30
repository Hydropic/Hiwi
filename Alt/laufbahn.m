function laufbahn(robot,pfad,runden,preserve_plot)
    hold on
    for j=1:runden
        %set(gcf,"Visible","on");
        robot.DataFormat = 'row';
        for i = 1:size(pfad,1)
            if preserve_plot
                %Diese Option zeichnet alle Stellungen der Bahn auf einem
                %Bild.
                %Praktisch um die Orientierung der Kelle zwischen zwei Schritten
                %besser zu vergleichen:
                show(robot,pfad(i,:),"Visuals","off");
            else
                %Diese Option lässt den Roboter die Bahn abfahren(praktisch
                %um zu überprüfen, ob die Punkte in der richtigen Reihenfolge
                %angesteuert werden:
                show(robot,pfad(i,:),"PreservePlot",false,"FastUpdate", true);
                pause(0.5);
            end
            drawnow;
            hold on
        end
        hold on
    end
end

