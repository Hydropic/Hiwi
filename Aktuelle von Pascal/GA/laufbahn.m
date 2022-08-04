function laufbahn(robot,pfad,runden,preserve_plot)
    hold on
    %anzahl, wie oft der weg durchlaufen werden soll
    for j=1:runden

        robot.DataFormat = 'row';
        tcppunkte = zeros(size(pfad,1),3);
        eulerZYX = zeros(size(pfad,1),3);
        eulerXYZ = zeros(size(pfad,1),3);

        %Für jedes Achsstellung des Pfades wird die Pose Geplottet
        for i = 1:size(pfad,1)

            %anwenden der Vorwärzkinematik um benötigte werte zu erhalten
            [tcppunkte(i,:),eulerZYX(i,:),eulerXYZ(i,:)] = vorwaertskinematik(pfad(i,:));

            %euler z sym ist die z Rotation von TCP_XY zu Welt -90° da die systeme um -90° zueiander verdreht sind            
            eulerZ_Sym = (eulerZYX(i,1)-90);
            if eulerZ_Sym < -180
                eulerZ_Sym = 180-abs(eulerZ_Sym + 180);
            elseif eulerZ_Sym > 180
                eulerZ_Sym = -180+abs(eulerZ_Sym - 180);
            end

            %Hin und Rücktransformation von TCP und Welt
            rot_TCP_zu_Welt =RotationUmZ(deg2rad(eulerZYX(i,1)))*RotationUmY(deg2rad(eulerZYX(i,2)))*RotationUmX(deg2rad(eulerZYX(i,3)));
            rot_Welt_Zu_TCP =transpose(RotationUmX(deg2rad(eulerZYX(i,3))))*transpose(RotationUmY(deg2rad(eulerZYX(i,2))))*transpose(RotationUmZ(deg2rad(eulerZYX(i,1))));   

            %Hin Und Rücktransformation von Sym und Welt
            %!!!!!!!!!!!!!!!!!!!!!Systeme um -90° zueiander gedreht!!!!!!!!!!!!!!!!!!                 
            rot_Sym_zu_Welt = RotationUmZ(deg2rad(eulerZ_Sym))*RotationUmY(deg2rad(0))*RotationUmX(deg2rad(0));
            rot_Welt_zu_Sym = transpose(RotationUmX(deg2rad(0)))*transpose(RotationUmY(deg2rad(0)))*transpose(RotationUmZ(deg2rad(eulerZ_Sym)));
            
            %b =[1;0;0]
            %b_ = rot_Sym_zu_Welt*b;
            %b__ = rot_Welt_zu_Sym*b_;

            %Berechnung der Vorzugsrichtung in Weltkoordinaten
            vorzugsrichtung_TCP = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
            vorzugsrichtung = rot_TCP_zu_Welt*vorzugsrichtung_TCP; % die Transformation von Global in TCP-Koordinaten

            %Debug der Transformationen
            %x = [1;0;1];
            %x_ = rot_Sym_zu_Welt*x;
            %x__ = rot_Welt_zu_Sym*x_,
            
            %Projektion der vorzugsrichtung auf xy-Ebene
            %Debug%Vorzugsrichtung_TCP = rot_Welt_Zu_TCP*Vorzugsrichtung;
            vorzugsrichtung_proj = [vorzugsrichtung(1),vorzugsrichtung(2),0]*1/sqrt(vorzugsrichtung(1)^2+vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor
                           
            %plottet vorzugsrichtung in Gelb in Weltkoordinaten            
            vorzugsR_Plot = plot3([tcppunkte(i,1),tcppunkte(i,1)+vorzugsrichtung(1)],...
            [tcppunkte(i,2),tcppunkte(i,2)+vorzugsrichtung(2)], ...
            [tcppunkte(i,3),tcppunkte(i,3)+vorzugsrichtung(3)],'Color','b','LineWidth',1);

            %plottet projezierte vorzugsrichtung in Rot in Weltkoordinaten                
            vorzRPr_Plot = plot3([tcppunkte(i,1),tcppunkte(i,1)+vorzugsrichtung_proj(1)],...
            [tcppunkte(i,2),tcppunkte(i,2)+vorzugsrichtung_proj(2)], ...
            [tcppunkte(i,3),tcppunkte(i,3)+vorzugsrichtung_proj(3)],'Color','magenta','LineWidth',2);
            tcp = transpose(tcppunkte(i,:));

            %Plotten des Korridors
            plotKorridor(tcp,vorzugsrichtung,rot_Welt_zu_Sym,rot_Sym_zu_Welt,eulerZYX(i,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%{Beibehalten der alten Posen}%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if preserve_plot                
                %Diese Option zeichnet alle Stellungen der Bahn auf einem
                %Bild.
                %Praktisch um die Orientierung der Kelle zwischen zwei Schritten
                %besser zu vergleichen:
                hold on

                %plotten des Roboters
                %!!!!!!!!!Achtung achswinkel in vorwaertskinematik gegenläufig!!!!!!!!!!!                
                show(robot,-pfad(i,:),"Visuals","off");

                %Pause um Bahn verfolgen zu Können
                pause(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%{Entfernen der alten Posen}%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            else
                %Diese Option lässt den Roboter die Bahn abfahren(praktisch
                %um zu überprüfen, ob die Punkte in der richtigen Reihenfolge
                %angesteuert werden:               
                               
                %!!!!!!!!!Achtung achswinkel in vorwaertskinematik gegenläufig!!!!!!!!!!
                show(robot,-pfad(i,:),"Visuals","off","PreservePlot",false);
                
                pause(1);

            end
            drawnow;
            hold on
        end
        hold on
    end
end

