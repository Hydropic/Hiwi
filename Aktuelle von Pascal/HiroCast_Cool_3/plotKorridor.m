function plotKorridor(TCP_Punkte,vorzugsrichtungPr,rot_WeltToSym,rot_SymToWelt,eulerXYZ)
        hold on              
        simu_CFD = importdata('simulationData.txt');
        simu_CFD = simu_CFD(:,1:9); % Alle Daten aus CFD_Simulation
        Ry_Rz_in_Sym = simu_CFD(:,1:2); % ALle Orientierung aus CFD_Simulation

        %%vorzugsrichtungPr = [vorzugsrichtungPr_W(1);vorzugsrichtungPr_W(2);vorzugsrichtungPr_W(3)];
        %%vorzugsrichtungPr = RotationUmX(0)*RotationUmY(0)*RotationUmZ(deg2rad(eulerXY(1,1)))*vorzugsrichtungPr_W;

        %Transformation des Vorzugsvektors in Sym-Systm
        vorzugsrichtungPr = rot_WeltToSym*transpose(vorzugsrichtungPr);
        vorzugsrichtungPr = transpose(vorzugsrichtungPr);

        %%%%%%%searchVector = [eulerXY(1,2),eulerXY(1,3),0,vorzugsrichtungPr(1,1),vorzugsrichtungPr(1,2),vorzugsrichtungPr(1,3),0,0];
        searchVector = [eulerXYZ(1,3),eulerXYZ(1,2),0,vorzugsrichtungPr(1,1),vorzugsrichtungPr(1,2),vorzugsrichtungPr(1,3),0,0];

        [k,dist] = dsearchn(simu_CFD(:,1:8),searchVector);
                                
        %%%%%%%%%%%%%%%%%%%%%% korridor in 3d darstellen %%%%%%%%%%%%%%%%%%%%%%%%%%    
        dx = TCP_Punkte(1,1);
        dy = TCP_Punkte(2,1);
        dz = TCP_Punkte(3,1);

        phi_1 = simu_CFD(k,7);
        
        %%%quiver3(dx,dy,dz,vorzugsrichtungPr(1,1),vorzugsrichtungPr(1,2),vorzugsrichtungPr(1,3),1,'g');

        hold on
        for  z=1:9
            vektor_z_Sym = simu_CFD(k+z-1,4:6);
            %transformation des korridors in Weltkoordinaten
            vektor_z = rot_SymToWelt*transpose(vektor_z_Sym);              
            quiver3(dx,dy,dz,(vektor_z(1,1)),(vektor_z(2,1)),(vektor_z(3,1)),1,'r');          
        end        
        end

