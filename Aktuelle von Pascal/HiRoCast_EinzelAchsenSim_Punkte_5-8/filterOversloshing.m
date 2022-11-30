function [smoothPendel_y, smoothPendel_x] = filterOversloshing(path_angular_deflection, saveNewPendel, editPendel)

    if saveNewPendel == true
        pendelkurve_edit = [path_angular_deflection(:,1), path_angular_deflection(:,2), path_angular_deflection(:,3)];
        save('Pendelausschlag.mat','pendelkurve_edit');
        figure;
        plot(pendelkurve_edit(:,1),pendelkurve_edit(:,2))
        figure;
        plot(pendelkurve_edit(:,1),pendelkurve_edit(:,3))
        smoothPendel = pendelkurve_edit;
    end

    if editPendel == true  
        pendelkurve = matfile('Pendelausschlag.mat');
        pendelkurve_geladen = pendelkurve.pendelkurve_edit;
%         plot(pendelkurve_geladen(:,1),pendelkurve_geladen(:,2)); 

        % setzt letzten Punkt = 0 
        [row, clm] = size(pendelkurve_geladen);
        pendelkurve_geladen(row, 2) = 0;
        pendelkurve_geladen(row, 3) = 0;

        %% ===== Glätten von Y ======
        % Händisches Überschwappen rausfiltern
%         ersterPunkt = 120;
%         letztePunkt = 131;
%         for dataPoint = ersterPunkt:letztePunkt
%             pendelkurve_geladen(dataPoint,2) = pendelkurve_geladen(ersterPunkt,2)+((pendelkurve_geladen(letztePunkt,2)-pendelkurve_geladen(ersterPunkt,2))/((letztePunkt-ersterPunkt)))*(dataPoint-ersterPunkt)
%         end
        pendelkurve_edit = pendelkurve_geladen;
%         plot(pendelkurve_edit(:,1),pendelkurve_edit(:,2));
        pendelkurve_edit(end,2) = 0;
        % save('Pendelausschlag.mat','pendelkurve_edit');
    
        %Kurve glätten
        num = (1/30)*[1 1 1 1 1 1 1 1 1 1 ...
            1 1 1 1 1 1 1 1 1 1 ...
            1 1 1 1 1 1 1 1 1 1];
        den = [1];
        newGeglaettet = filter(num,den,pendelkurve_edit(:,2));
%         figure
%         plot(newGeglaettet)
        % save('Pendelausschlag.mat','pendelkurve_edit');

        smoothPendel_y = smoothdata(newGeglaettet);
%         figure
%         plot(smoothPendel_y)

        %% ===== Glätten von X ======
        pendelkurve_geladen2 = pendelkurve.pendelkurve_edit;
        % Händisches Überschwappen rausfiltern
%         ersterPunkt2 = 120;
%         letztePunkt2 = 131;
%         for dataPoint2 = ersterPunkt2:letztePunkt2
%             pendelkurve_geladen2(dataPoint2,2) = pendelkurve_geladen(ersterPunkt2,3)+((pendelkurve_geladen(letztePunkt2,3)-pendelkurve_geladen(ersterPunkt2,3))/((letztePunkt2-ersterPunkt2)))*(dataPoint2-ersterPunkt2)
%         end
        pendelkurve_edit2 = pendelkurve_geladen2;
%         figure;
%         plot(pendelkurve_edit2(:,1),pendelkurve_edit2(:,3));
        pendelkurve_edit2(end,3) = 0;
        % save('Pendelausschlag.mat','pendelkurve_edit');
    
        %Kurve glätten
        num = (1/30)*[1 1 1 1 1 1 1 1 1 1 ...
            1 1 1 1 1 1 1 1 1 1 ...
            1 1 1 1 1 1 1 1 1 1];
        den = [1];
        newGeglaettet2 = filter(num,den,pendelkurve_edit2(:,3));
%         figure
%         plot(newGeglaettet2)
        % save('Pendelausschlag.mat','pendelkurve_edit');

        smoothPendel_x = smoothdata(newGeglaettet2);
%         figure
%         plot(smoothPendel_x)
    end
end

