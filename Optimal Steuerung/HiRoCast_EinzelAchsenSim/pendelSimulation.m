function [pendelBahn] = pendelSimulation(optimized_translational_values)

[optimized_translational_values_sumTime] = sumTimeAndMM(optimized_translational_values);

% hinzufügen von einer weiteren Sekunde Simulationszeit
for t = 1:100
optimized_translational_values_sumTime(end+1,:) = [optimized_translational_values_sumTime(end,1)+0.1 , optimized_translational_values_sumTime(end,2), optimized_translational_values_sumTime(end,3), optimized_translational_values_sumTime(end,4)]
end

g = 9.81;   %[m/s^2]
L = 0.0374;    %[m] für reelle f=2.05-> 0.059129 [m],
M = 0.164;      %[kg] 125g--> 107g Wasser 158g Volumen"
d = 0.0;    %[N/(m/s)]

% d = 0.0621;    %[N/(m/s)] % Dempfungskonstante von Daniel
% d = 1.51 [N/m/s] Paper Reinhold

dt=0.01;   %[s]

% Winkelgrößen in RAD
%=========== Ünersicht: Indizierung der Wertematrix ===================
A =      [   0,      0,       0,        0,      0,      0,      0,...
    ...Header=["Zeit", "theta", "vtheta", "atheta", "phi", "vphi", "aphi",
    ...Hindex = [ 1,      2,       3,        4,       5,      6,     7,
    ...
            0,       0,       0,          0,      0 ,    0,    0,   0,    0 ...
    ... "theta_S","phi_S",  "Sl.A._S ", "x_P",  "y_P", "z_P", "x", "vx", "ax",
    ...     8,       9,       10,         11,     12,    13,   14,  15,   16,
    ...
          0,     0,     0,     0,    0,    0,    0,   0,   0,    0 ...
    ...  "y",   "vy",  "ay",  "z",  "vz", "az", "a", "b", "c", "Lnorm",...
    ...  17,     18,    19,    20,   21,   22,   23,  24,  25,   26
    ...
          0,     0,    0,        0];
    ...  "vB", "azp", "wB", "SloshingAngle"]
    ...   27,   28,    29,       30]  ;

optimized_translational_values_sumTime(:,2:end)= optimized_translational_values_sumTime (:,2:end)*0.001; %Umrechnung von mm in m da mit SI einheiten gerechnet wird

[v] = Ableiten(optimized_translational_values_sumTime);
[a] = Ableiten(v);

[Ortsdaten_I] = Interpolieren(optimized_translational_values_sumTime,dt);
[v_I] = Interpolieren(v,dt);
[a_I] = Interpolieren(a,dt);

A(1:length(Ortsdaten_I),[14,17,20]) = Ortsdaten_I(:,2:4);
A(1:length(v_I),[15,18,21]) = v_I(:,2:4);
A(1:length(a_I),[16,19,22]) = a_I(:,2:4);

Zeilen = length(A);

% =============== Iterative Berechnung der Ergebnismatrix =================
j=1;
while j<=Zeilen
    A(j+1,1)=A(j,1)+dt;
    % ============ Bewegungsgleichungen Pendelwinkel ======================
    %     A(j+1,4) = -((g+A(j,22))/(L))*sind(A(j,2))+(A(j,16)/(L))*cosd(A(j,2))...
    %                 -(d/M)*A(j,3)*(cosd(A(j,2)))^2; % theta°°
    % linearisiert
    A(j+1,4) = -((g+A(j,22))/(L))*A(j,2)+(A(j,16)/(L))...
        -(d/M)*A(j,3); % theta°°
    A(j+1,3) = A(j+1,4)*dt+A(j,3);              % theta°
    A(j+1,2) = A(j+1,3)*dt+A(j,2);              % theta in Radiant

    %     A(j+1,7) = -((g+A(j,22))/(L))*sind(A(j,5))+(A(j,19)/(L))*cosd(A(j,5))...
    %                 -(d/M)*A(j,6)*(cosd(A(j,5)))^2; % phi°°
    % linearisiert
    A(j+1,7) = -((g+A(j,22))/(L))*A(j,5)+(A(j,19)/(L))...
        -(d/M)*A(j,6); % phi°°
    A(j+1,6) = A(j+1,7)*dt+A(j,6);              % phi°
    A(j+1,5) = A(j+1,6)*dt+A(j,5);              % phi in Radiant
   
    % Folgegrößen errechnen für die erste Zeile aus den Anfangsbedingungen
    % berrechnen
    if j==1
        % Nur für räumliche Visualisierung wichtig

        % Normalenvektoren mit der Zeit
        % für den Startwert und ersten Folgeschritt berechnet

        A(j:j+1,23) = cos(A(j:j+1,5)).*sin(A(j:j+1,2)); %Punktoperator für Elementeweise Multiplikation
        A(j:j+1,24) = sin(A(j:j+1,5)).*cos(A(j:j+1,2));
        A(j:j+1,25) = cos(A(j:j+1,5)).*cos(A(j:j+1,2));
        A(j:j+1,26) = sqrt(A(j:j+1,23).^2+A(j:j+1,24).^2+A(j:j+1,25).^2);
        % x_P
        A(j:j+1,11) =- L/A(j:j+1,26) * A(j:j+1,23);
        % y_P
        A(j:j+1,12) =- L/A(j:j+1,26) * A(j:j+1,24);
        % z_P
        A(j:j+1,13) =- L/A(j:j+1,26) * A(j:j+1,25);

       
    else
        % für alle anderen Folgeschritte
        A(j+1,23) = cos(A(j+1,5))*sin(A(j+1,2));
        A(j+1,24) = sin(A(j+1,5))*cos(A(j+1,2));
        A(j+1,25) = cos(A(j+1,5))*cos(A(j+1,2));
        A(j+1,26) = sqrt(A(j+1,23)^2+A(j+1,24)^2+A(j+1,25)^2);
        % x_P
        A(j+1,11) =- L/A(j+1,26) * A(j+1,23);
        % y_P
        A(j+1,12) =- L/A(j+1,26) * A(j+1,24);
        % z_P
        A(j+1,13) =- L/A(j+1,26) * A(j+1,25);
        %         % Sloshing Angle (Winkel Pendelstab zur Z-Achse
                 zv=[0;0;-1];
                 v=[A(j+1,11);A(j+1,12);A(j+1,13)];
                 A(j+1,30)=acosd(norm(dot(zv,v))/(norm(zv)*norm(v)));

    end
    j=j+1; %Nächster Berechnungsschritt
end


% Ergebnis = Zeit, Winkel X-Richtung, Winkel Z-Richtung
pendelBahn= A(:,[1,2,5]);
pendelBahn(:,2:end)= pendelBahn (:,2:end)*360/(2*pi); %RAD in Grad

end

function [Erg] = Ableiten(data)
Erg = zeros(length(data),width(data));
Erg(:,1) = data(:,1);
j=1;        % v=ds/dt oder a=dv/dt
while j<length(Erg)
    dt = data(j+1,1)-data(j,1);
    Erg(j+1,2)= (data(j+1,2)-data(j,2))/dt ;
    Erg(j+1,3)= (data(j+1,3)-data(j,3))/dt ;
    Erg(j+1,4)= (data(j+1,4)-data(j,4))/dt ;
    j=j+1;
end
end

function [Erg] = Interpolieren (data,dt)
% Matrix für Anpassung der Zeiten der Wertematrix und der Sensor/GOM Matrix und Interpolation der Zwischenwerte
tgesData = data(end,1);
idxNew= ceil(tgesData/dt);
tgesNew = idxNew*dt;
Zeit = 0:dt:tgesNew;

Erg = zeros(length(Zeit),width(data));
Erg(:,1) = Zeit; % Zeiten aus A für GOM übernehmen
% Zu interpolierende Daten fangen immer bei ~=0 an, daher wird für den
% Zeitpunkt 0 der Interpolierten Werte der erste Datensatz der zu
% interpolierenden Daten übernommen
Erg(1,2:end)= data(1,2:end);
% GOM Datenreihe auf A-Zeiten zuordnen und Zwischenwerte Interpolieren
j = 1; % Aktuelle Zeile in Zielmatrix (GOM_INT, größere Matrix)
lj= 1; % Zeile in GOM_INT des letzten zugeordneten GOM-Wertes
skip=false;
k = 1; % Aktueller Zeile in zuzuordnender Matrix (GOM_G)
kmax = length(data);
while j<=length(Erg) %
    if kmax < k  % Abbruch wenn letzter GOM Wert überschritten
        break
    elseif data(k,1)<=Erg(j,1) % mit gleich oder kleiner gleich gab es Probleme
        % NaN sorgt für "Stufen" in der Interpolation
        if isnan(data(k,2))==true || isnan(data(k,3))==true || isnan(data(k,4))==true
            skip=true;
        end
        if skip==false
            Erg(j,2) = data(k,2);
            Erg(j,3) = data(k,3);
            Erg(j,4) = data(k,4);
            % Interpolation
            AnzInt = j-lj;               % Bereich über den Interpoliert wird
            m1 = (Erg(j,2)-Erg(lj,2))/(j-lj); % Steigung 1
            m2 = (Erg(j,3)-Erg(lj,3))/(j-lj); % Steigung 2
            m3 = (Erg(j,4)-Erg(lj,4))/(j-lj); % Steigung 3
            % Fehlerabfang bei Start bei 0-Werten
            if isnan(m1)==true, m1=0; end
            if isnan(m2)==true, m2=0; end
            if isnan(m3)==true, m3=0; end
            for idx = 0:AnzInt
                Erg(lj+idx,2) = Erg(lj,2)+m1*idx; % Geradengleichung für Interp.
                Erg(lj+idx,3) = Erg(lj,3)+m2*idx;
                Erg(lj+idx,4) = Erg(lj,4)+m3*idx;
            end
            % Ende Interpolation
            lj = j;
        else
            skip=false;
        end
        k=k+1;
    end
    j=j+1;
end
end