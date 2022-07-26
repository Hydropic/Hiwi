function laufbahn(robot,pfad,runden,preserve_plot)
    hold on
    for j=1:runden
        %set(gcf,"Visible","on");
        robot.DataFormat = 'row';
        tcppunkte = zeros(size(pfad,1),3);
        eulerZYX = zeros(size(pfad,1),3);
        for i = 1:size(pfad,1)
            if preserve_plot
                
                %Diese Option zeichnet alle Stellungen der Bahn auf einem
                %Bild.
                %Praktisch um die Orientierung der Kelle zwischen zwei Schritten
                %besser zu vergleichen:
                [tcppunkte(i,:),eulerZYX(i,:)] = vorwaertskinematik(pfad(i,:));
                hold on
                plot3(tcppunkte(i,1),tcppunkte(i,2),tcppunkte(i,3), "Marker", ".")
            
                rot_Welt_zu_TCP = inv(RotationUmZ(deg2rad(-eulerZYX(i,1))))*inv(RotationUmY(deg2rad(-eulerZYX(i,2))))*inv(RotationUmX(deg2rad(-eulerZYX(i,3)))); 
                Vorzugsrichtung_global = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
                Vorzugsrichtung = rot_Welt_zu_TCP*Vorzugsrichtung_global; % die Transformation von Global in TCP-Koordinaten
                Vorzugsrichtung_proj = [Vorzugsrichtung(1),Vorzugsrichtung(2),0]*0.3/sqrt(Vorzugsrichtung(1)^2+Vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor
                            
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung(3)],'Color','r','LineWidth',1);
                show(robot,pfad(i,:),"Visuals","off");

                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung_proj(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung_proj(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung_proj(3)],'Color','r','LineWidth',1);
                show(robot,pfad(i,:),"Visuals","off");
            else
                %Diese Option lässt den Roboter die Bahn abfahren(praktisch
                %um zu überprüfen, ob die Punkte in der richtigen Reihenfolge
                %angesteuert werden:
                [tcppunkte(i,:),eulerZYX(i,:)] = vorwaertskinematik(pfad(i,:));
                hold on
                plot3(tcppunkte(i,1),tcppunkte(i,2),tcppunkte(i,3), "Marker", ".")
            
                rot_Welt_zu_TCP = RotationUmZ(deg2rad(-eulerZYX(i,3)))*RotationUmY(deg2rad(-eulerZYX(i,2)))*RotationUmX(deg2rad(-eulerZYX(i,1))); 
                Vorzugsrichtung_global = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
                Vorzugsrichtung = rot_Welt_zu_TCP*Vorzugsrichtung_global; % die Transformation von Global in TCP-Koordinaten
                Vorzugsrichtung_proj = [Vorzugsrichtung(1),Vorzugsrichtung(2),0]*0.3/sqrt(Vorzugsrichtung(1)^2+Vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor
                            
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung_proj(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung_proj(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung_proj(3)],'Color','r','LineWidth',5);
                hold off
                show(robot,pfad(i,:),"PreservePlot",false,"FastUpdate", true);
                pause(0.5);
            end
            drawnow;
            hold on
        end
        hold on
    end
end

