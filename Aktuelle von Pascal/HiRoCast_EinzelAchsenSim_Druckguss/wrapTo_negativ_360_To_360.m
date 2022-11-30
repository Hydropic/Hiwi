function [angels] = wrapTo_negativ_360_To_360(anglesWrapedTo180)
    %Diese Funktion wandelt winkel die als winkel von -180 bis 180 angegeben
    %werden als stetigen verlauf von -360 bis 360 da

    angels = anglesWrapedTo180;

    %Bestimmen der Punkte an denen ein sprung vorliegt
    sizeAng = size(anglesWrapedTo180,1);
    Neg180ToPos180 = zeros(sizeAng,1);
    Pos180ToNeg180 = zeros(sizeAng,1);
    switchdSign = zeros(sizeAng,1);
    firstSign = sign(anglesWrapedTo180(1,1));
    for i = 2:sizeAng
        diff = anglesWrapedTo180(i-1) - anglesWrapedTo180(i);
        if diff > 345
            Neg180ToPos180(i,1) = 1;
            switchdSign(i,1) = 1;
        elseif diff < -345
            Pos180ToNeg180(i,1) = 1;
            switchdSign(i,1) = -1;
        end
    end

    relocating = 0;

    for j = 1:sizeAng
        if switchdSign(j) ~= 0
            if switchdSign(j) > 0 && relocating == 0
                angels(j) = anglesWrapedTo180(j) + 360;
                relocating = -1;

            elseif switchdSign(j) < 0 && relocating == 0
                angels(j) = anglesWrapedTo180(j) - 360;
                relocating = 1;

            else
                relocating = 0;

            end
        elseif relocating == -1 && switchdSign(j) == 0
            angels(j) = anglesWrapedTo180(j) + 360;

        elseif relocating == 1  && switchdSign(j) == 0           
            angels(j) = anglesWrapedTo180(j) - 360;

        end  
    end
%BUG 
% % %     %Filter Values <-360 abd >360
% % %     for k = 1:1:sizeAng
% % %         if angels(k) > 360
% % %             angels(k) = angels(k)- 360*2;
% % %         end
% % % 
% % %         if angels(k) < -360
% % %             angels(k) = angels(k)+ 360*2;
% % %         end
% % %     end    
end