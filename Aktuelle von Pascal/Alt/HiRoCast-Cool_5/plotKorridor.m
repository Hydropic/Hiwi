function plotKorridor(TCP_Punkte,vorzugsrichtung,rot_WeltToSym,rot_SymToWelt,eulerZYX)

        hold on              
        simu_CFD = importdata('simulationData.txt');
        simu_CFD = simu_CFD(:,1:9); % Alle Daten aus CFD_Simulation

        %Vorzugsrichtung auf xy-Ebene projezieren und Normalisieren (länge)=1
        vorzugsrichtung = transpose(vorzugsrichtung);
        vorzugsrichtungPr = [vorzugsrichtung(1),vorzugsrichtung(2),0]*1/sqrt(vorzugsrichtung(1)^2+vorzugsrichtung(2)^2);

        %Transformation des Vorzugsvektors in Sym-Systm
        vorzugsrichtung_sym = rot_WeltToSym*transpose(vorzugsrichtung);
        X_richtung_Sym = [1;0;0];
        Y_richtung_Sym = [0;1;0];
        
        %Rotationen zwischen Sym und dem TCP-XY system
        rot_SymToTCP_XY = RotationDegUmZ(-90);
        rot_TCP_XY_ToSYM = RotationDegUmZ(90);

        %Hilfsgrößen um transformation ins sym system durchzuführen/verstehen        
        %X_richtung_Sym_proj_TCP = rot_SymToTCP_XY*X_richtung_Sym;
        %Y_richtung_Sym_proj_TCP = rot_SymToTCP_XY*Y_richtung_Sym;

        %Hilfsgrößen um winkel im sym system zu berechnen
        %X_richtung_Sym_ = rot_TCP_XY_ToSYM*X_richtung_Sym_proj_TCP;
        %Y_Richtung_Sym_ = rot_TCP_XY_ToSYM*Y_richtung_Sym_proj_TCP;

        %Die Projezierte vorzugsrichtung in das Simulationssystem drehen
        vorzugsrichtungPr_sym = rot_TCP_XY_ToSYM*transpose(vorzugsrichtungPr);
        vorzugsrichtungPr_sym = transpose(vorzugsrichtungPr_sym);

        %die Winkel eulerX und eulerY müssen angepasst oder neu berechnet werden      
        %eulerX_Sym = atand(vorzugsrichtung_sym(2,1)/vorzugsrichtung_sym(3,1));
        %eulerY_Sym = atand(vorzugsrichtung_sym(1,1)/vorzugsrichtung_sym(3,1));

        %-eulerZYX(1,2) entspricht der xDrehung im sym system!!!!
        searchVector = [-eulerZYX(1,2),eulerZYX(1,3),0,vorzugsrichtungPr_sym(1,1),vorzugsrichtungPr_sym(1,2),vorzugsrichtungPr_sym(1,3),0,0];
        
        %Suche nach einem Passenden eintrag in der Tabelle
        [k,dist] = dsearchn(simu_CFD(:,1:8),searchVector);
                                    
        dx = TCP_Punkte(1,1);
        dy = TCP_Punkte(2,1);
        dz = TCP_Punkte(3,1);
               
        %korridor in 3d darstellen 
        hold on
        for  z=1:9
            vektor_z_Sym = simu_CFD(k+z-1,4:6);
            %transformation des korridors in Weltkoordinaten
            vektor_z = rot_SymToWelt*transpose(vektor_z_Sym);              
            quiver3(dx,dy,dz,(vektor_z(1,1)),(vektor_z(2,1)),(vektor_z(3,1)),1,'color','r','LineWidth',1);          
        end        
end
