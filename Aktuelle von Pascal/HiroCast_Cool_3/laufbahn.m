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
                           
                %%%%%% rot_TCP_zu_Welt = inv(RotationUmZ(deg2rad(-eulerZYX(i,1))))*inv(RotationUmY(deg2rad(-eulerZYX(i,2))))*inv(RotationUmX(deg2rad(-eulerZYX(i,3))));

                %Hin und Rücktransformation von TCP und Welt
                rot_TCP_zu_Welt =RotationUmZ(deg2rad(eulerZYX(i,1)))*RotationUmY(deg2rad(eulerZYX(i,2)))*RotationUmX(deg2rad(eulerZYX(i,3)));
                rot_Welt_Zu_TCP =transpose(RotationUmX(deg2rad(eulerZYX(i,3))))*transpose(RotationUmY(deg2rad(eulerZYX(i,2))))*transpose(RotationUmZ(deg2rad(eulerZYX(i,1))));   

                %Hin Und Rücktransformation von Sym und Welt
                rot_Sym_zu_Welt = RotationUmZ(deg2rad(eulerZYX(i,1)))*RotationUmY(deg2rad(0))*RotationUmX(deg2rad(0));
                rot_Welt_zu_Sym = transpose(RotationUmX(deg2rad(0)))*transpose(RotationUmY(deg2rad(0)))*transpose(RotationUmZ(deg2rad(eulerZYX(i,1))));

                %Berechnung der Vorzugsrichtung in Weltkoordinaten
                Vorzugsrichtung_TCP = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
                Vorzugsrichtung = rot_TCP_zu_Welt*Vorzugsrichtung_TCP; % die Transformation von Global in TCP-Koordinaten

%Debug der Transformationen
% % % % %                 x = [1;0;1];
% % % % %                 x_ = rot_Sym_zu_Welt*x;
% % % % %                 x__ = rot_Welt_zu_Sym*x_,
                
                %Projektion der vorzugsrichtung auf xy-Ebene
%Debug%         Vorzugsrichtung_TCP = rot_Welt_Zu_TCP*Vorzugsrichtung;
                Vorzugsrichtung_proj = [Vorzugsrichtung(1),Vorzugsrichtung(2),0]*1/sqrt(Vorzugsrichtung(1)^2+Vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor

                %plottet vorzugsrichtung in Gelb in Weltkoordinaten            
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung(3)],'Color','b','LineWidth',1);

                %plottet projezierte vorzugsrichtung in Rot in Weltkoordinaten                
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung_proj(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung_proj(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung_proj(3)],'Color','r','LineWidth',2);
                tcp = transpose(tcppunkte(i,:));
                Vorzugsrichtung_proj = RotationUmZ(eulerZYX(1,1))*transpose(Vorzugsrichtung_proj);
                Vorzugsrichtung_proj = transpose(Vorzugsrichtung_proj);
                
                %Plotten des Korridors
                plotKorridor(tcp,Vorzugsrichtung_proj,rot_Welt_zu_Sym,rot_Sym_zu_Welt,eulerZYX(i,:));

                %plotten des Roboters
                %!!!!!!!!!Achtung achswinkel in vorwaertskinematik gegenläufig!!!!!!!!!!!                
                show(robot,-pfad(i,:),"Visuals","off");

                %Pause um Bahn verfolgen zu Können
                pause(0.5);
                
            else
                %Diese Option lässt den Roboter die Bahn abfahren(praktisch
                %um zu überprüfen, ob die Punkte in der richtigen Reihenfolge
                %angesteuert werden:
                                [tcppunkte(i,:),eulerZYX(i,:)] = vorwaertskinematik(pfad(i,:));
                hold on
                plot3(tcppunkte(i,1),tcppunkte(i,2),tcppunkte(i,3), "Marker", ".")
                           
                %%%%%% rot_TCP_zu_Welt = inv(RotationUmZ(deg2rad(-eulerZYX(i,1))))*inv(RotationUmY(deg2rad(-eulerZYX(i,2))))*inv(RotationUmX(deg2rad(-eulerZYX(i,3))));

                %Hin und Rücktransformation von TCP und Welt
                rot_TCP_zu_Welt =RotationUmZ(deg2rad(eulerZYX(i,1)))*RotationUmY(deg2rad(eulerZYX(i,2)))*RotationUmX(deg2rad(eulerZYX(i,3)));
                rot_Welt_Zu_TCP =transpose(RotationUmX(deg2rad(eulerZYX(i,3))))*transpose(RotationUmY(deg2rad(eulerZYX(i,2))))*transpose(RotationUmZ(deg2rad(eulerZYX(i,1))));   

                %Hin Und Rücktransformation von Sym und Welt
                rot_Sym_zu_Welt = RotationUmZ(deg2rad(eulerZYX(i,1)))*RotationUmY(deg2rad(0))*RotationUmX(deg2rad(0));
                rot_Welt_zu_Sym = transpose(RotationUmX(deg2rad(0)))*transpose(RotationUmY(deg2rad(0)))*transpose(RotationUmZ(deg2rad(eulerZYX(i,1))));

                %Berechnung der Vorzugsrichtung in Weltkoordinaten
                Vorzugsrichtung_TCP = [0;0;1];% Vorzugsrichtung zeigt in TCP-Z
                Vorzugsrichtung = rot_TCP_zu_Welt*Vorzugsrichtung_TCP; % die Transformation von Global in TCP-Koordinaten

%Debug der Transformationen
% % % % %                 x = [1;0;1];
% % % % %                 x_ = rot_Sym_zu_Welt*x;
% % % % %                 x__ = rot_Welt_zu_Sym*x_,
                
                %Projektion der vorzugsrichtung auf xy-Ebene
                Vorzugsrichtung_TCP = rot_Welt_Zu_TCP*Vorzugsrichtung;
                Vorzugsrichtung_proj = [Vorzugsrichtung(1),Vorzugsrichtung(2),0]*1/sqrt(Vorzugsrichtung(1)^2+Vorzugsrichtung(2)^2);%normierter Proj Vorzugsrichtungsvektor

                %plottet vorzugsrichtung in Gelb in Weltkoordinaten            
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung(3)],'Color','b','LineWidth',1);

                %plottet projezierte vorzugsrichtung in Rot in Weltkoordinaten                
                plot3([tcppunkte(i,1),tcppunkte(i,1)+Vorzugsrichtung_proj(1)],...
                [tcppunkte(i,2),tcppunkte(i,2)+Vorzugsrichtung_proj(2)], ...
                [tcppunkte(i,3),tcppunkte(i,3)+Vorzugsrichtung_proj(3)],'Color','r','LineWidth',2);
                tcp = transpose(tcppunkte(i,:));
                Vorzugsrichtung_proj = RotationUmZ(eulerZYX(1,1))*transpose(Vorzugsrichtung_proj);
                Vorzugsrichtung_proj = transpose(Vorzugsrichtung_proj);
                
                %Plotten des Korridors
                plotKorridor(tcp,Vorzugsrichtung_proj,rot_Welt_zu_Sym,rot_Sym_zu_Welt,eulerZYX(i,:));
                hold off
                
                %!!!!!!!!!Achtung achswinkel in vorwaertskinematik gegenläufig!!!!!!!!!!
                show(robot,pfad(i,:),"PreservePlot",false,"FastUpdate", true);
                pause(0.5);
            end
            drawnow;
            hold on
        end
        hold on
    end
end

