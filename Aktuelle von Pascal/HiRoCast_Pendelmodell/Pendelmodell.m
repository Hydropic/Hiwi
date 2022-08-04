% Benötigte Add-ons
% Signal Processing Toolbox
% Curve Fitting Tool 3.6
clear all
format shortG
get(0,'factory');
set(0,'defaultfigurecolor',[1 1 1]);
flag_break_OS=0;
flag_break_AS=0;
Update_GOM_Daten = 1;
Update_S_Daten = 1;
% [S_Name,Sensor_Pfad]=uigetfile('*.csv','Wähle Sensordaten aus einem Sensorprofilordner aus','C:\Users\Daniel Meyer\sciebo\Projektarbeit\Pendelmodell zur Fluessigkeitssymulation\Messungen\Oberflächenmesseung\'); % default Ordner ausgewählt
% data_S_blank = readmatrix(Sensor_link);
% [GOM_Name,GOM_Pfad]=uigetfile('*.csv','Wähle Bewegungsdatei aus','C:\Users\Daniel Meyer\sciebo\Projektarbeit\Pendelmodell zur Fluessigkeitssymulation\Messungen\Beschleunigungsmessung\'); % default Ordner ausgewählt
% data_GOM = readmatrix(GOM_link);
%%
close all


%==========================
% ====Modellparameter======
%==========================
g = 9.81;   %[m/s^2]
L = 1;%0.0367; %0.057523;   %[m] für reelle f=2.05-> 0.059129 [m],
M = 1;%0.164;      %[kg] 125g--> 107g Wasser 158g Volumen"
d = 0;%0.0827;    %[N/(m/s)]
%==========================
%===Simulationsparameter===
%==========================
dt=0.01;   %[s]
FT=0.4; %[s/Datenpunkt Dargestellt] FrameTime mind. 0.15 bei Simulation, Anpassbar
tges= 15; %[s]

% Impulsfunktion
eta = 0.1;
% Wertebereich Ergebnismatrix
A = zeros((tges/dt+1),30);
A(:,1) = (0:dt:tges)';
B = (-L*1.1)*ones(length(A),1); % Für Bodenprojektion

% % Zeit, Z1, Z2, Z3, thetaS, phiS
% Vektor für Zeit der Berechnungsschritte
T=zeros(1,1);
% Winkelgrößen in RAD
%=========== Ünersicht: Indizierung der Wertematrix ===================
Header =      [   0,      0,       0,        0,      0,      0,      0,...
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

A(1,:)=Header;

% Einstellbare Anfangswerte Index:
% 2:7 Auslenkungen/Anfangswerte Pendel
% 14  Startpunkt im Raum X/Y/Z, weg durch Wegfunktion unten beschrieben
% 17
% 20

% ============================ Flag Variablen ===============================
%  1 = An
%  0 = Aus
Ermitteln         = 0;
Simulieren        = 1;
Animation         = 1;

CamMoving         = 1;
    Blickposition = ([-1 -2 1]);% x-y-z Vektor auf Pendelpunkt/ Ursprung
    n             = L; % Bereichsgröße [m] um Pendelpunkt/ Ursprung

SloshingAngle     = 0;      %Im Winkelamplitudenverlau wird der Winkel gegenüber der Z-Achse angezeigt
Zylinder          = 1;    % Darstellung der Flüssigkeit im Zylinder
    R             = 0.075;  % Radius Behälter
    H             = 0.075;  % Höhe Flüssigkeitsspiegel in Ruhe
SIM_Winkel_S      = 0;  % 1 = Winkel aus Sensormessung, statt der aus der Simulation wird Animiert (Nur wen Animation auch an)


GOM_Einbinden     = 0;
    PivotLinear   = 1;
    PivotCircular = 0;

Endergebnis       = 0;  % Keine Animation, nur Wegverlauf Pendel aufgezeichnet
Pendelposition    = 0;  % Nur bestimmte Pendelposition (aus Anfangswerten) anzeigen
ZeitAN            = 1;  % Berechnungslänge für einzelne Animationsframes anzeigen

% ======================== Daten einlesen ================================

% Sensor
ds = 32.103; %[mm]  % Abstand der Sensoren vom Mittelpunkt
mw_S     = 1; % Menge der Datenpunkte, über die gemittelt wird
Wdhl_S   = 1; % Wie oft der Glättvorgang mit mw_S Intervall wiederholt wird
S        = 2; % 2 = Vorzugsrichtung; 3= Nebenrichtung
% Spalte in welcher der zu analysierende Winkelverlauf steht

% Aus Zeitpunkten der ursprünglichen Datei
tshift_SA = 0;  % [s]  Anfang stutzen und zum Kallibrieren
tshift_SE = 0;  % [s]  Endpunkt festlegen
% Zeitpunkt der gestutzten Datei
tshift_Bew = 0; % 

% GOM
Bewegungsart = 'linear'; %GOM:    koord, linear, orthogonal, z-negativ, z-positiv,
                   % Emily: linear, neben, z-positiv, z-negativ
tshift_GOM = -tshift_Bew; % [s]
GOM_Hz = 0;  % Messfrequenz GOM-System

%==================Allgemeine Pendel Parameter====================
% w=sqrt(g/L);% Eigenkreisfrequenz w=2*pi*f
% delta=d/(2*M*w); % Dämpfungsgrad /-maß
% dek=w*delta; % logarithmisches Dekrement / Abklingkonstante
% w_d=sqrt(g/L-delta^2);
% f=w/(2*pi);
% f_d=w_d/(2*pi);
% % TP=1/fP;0.20061
% f_S = 2.05;
% L_S=g/(f_S*2*pi)^2;
% T=1/f;
% % PT2 Glied
% zeta = d/(2*M*w);
% wzl = sqrt(1-zeta^2); % Wurzelterm der PT impulsantwort

%==== Schleife die es ermöglicht der Übersicht halber den Code zu falten===
    CodeFalten =1;
% =============== manuelle Erstellung von Bewegungsabläufen =============

%=================Einstellungen lineare Strecke============================
while CodeFalten == 1
    % Strecke       % Wenn Strecke negativ, v  muss auch negativ sein
    P1 = [0; 0; 0];  % Startpunkt     %[1.618; -2.117; 1.218];
    P2 = [1; 0; 0];  % Endpunkt    %[ 1.616; -0.914; 1.220]; GOM X0.804

    sgesX= P2(1)-P1(1);       % (a passt sich an)
    sgesY= P2(2)-P1(2);
    sgesZ= P2(3)-P1(3);
    % maximale Geschwindigkeiten
    vmaxX = 1.4; % Geschwindigkeit * Prozent roboter
    vmaxY = 0;      % 1.2 *0.5
    vmaxZ = 0;
    % maximale Beschleunigung
    % amax vorgeben, wenn nicht vorgegeben werden soll (NULL) kann die Beschleunigungs-
    % zeit angegeben werden, amax wird dann berrechnet
    amaxX = 0;
        taX2 = 0.5; %GOM_w-GOM_a;
    amaxY = 0;
        taY2 = 0; %0.6
    amaxZ = 0;
        taZ2 = 0;
    % Zeitpunkt Beginn der Beschleunigung
    t1X = 2;%GOM_a;
    t1Y = 0; % = tmaxX weiter unten überschreiben für Eck-Bewegung
    t1Z = 0;
break
end
%====================Einstellungen Kreisbahn==============================
while CodeFalten == 1
    % r = 1;              % Radius
    vmaxB = 1;          % Bahngeschwindigkeit
    amaxB = 5.1;

    % Ortspunkte
    PM = ([-1; 0; 0;]); %Mittelpunkt
    PS = ([-1; -1; 0;]); % Startpunkt

    aBogen = 360; % Grad
    t1B=3;  % Begin der Beschleunigung
    taB=3;  % Dauer der Beschleunigung
break
end
%=================================================
%=================== GOM System ==================
%=================================================

if GOM_Einbinden==1
    if Update_GOM_Daten==1
        [GOM_Name,GOM_Pfad]=uigetfile('*.csv','Wähle Bewegungsdatei aus','C:\Users\Daniel Meyer\sciebo\Projektarbeit\Pendelmodell zur Fluessigkeitssymulation\Messungen\Beschleunigungsmessung\'); % default Ordner ausgewählt
        Update_GOM_Daten=0;
    end
    GOM_link=strcat(GOM_Pfad,GOM_Name);
    % Auswahl des Sensors für spätere Berechnung
    AS_GOM=contains(GOM_link,'GOM');
    AS_Emily = contains(GOM_link,'Emily');
    if AS_GOM ==1
        AS_Ordner = 'GOM';
    elseif AS_Emily==1
        AS_Ordner = 'Emily';
    else
        AS_Ordner ='';
        flag_break_OS =1;
    end
    if flag_break_OS ==1
        disp('Kein Bewegungspfrofilordner ausgewählt oder Vorgang abgebrochen')
    else

    data_GOM = readmatrix(GOM_link);
    % Fill weil Beschleunigungen evtl. nicht für 3 Richtungen ausgegeben werden
    % data_GOMFILL = zeros(length(data_GOM),4);
    % data_GOMFILL(:,1:width(data_GOM)) = data_GOM(:,1:width(data_GOM));
    % data_GOMFILL(:,2:end) = data_GOMFILL(:,2:end)*0.001;% [mm] in [m] umrechnen
    % data_GOM=data_GOMFILL;
    GOM = data_GOM;
    GOM (:,2:end)= GOM(:,2:end)*0.001; % [mm] in [m] umrechnen

    [GOM, Bewegungszeit] = BewegungsartZuordnen(GOM, GOM_Name, Bewegungsart);
    
    dt_GOM = 1/GOM_Hz;
    if GOM_Hz ~= 0
        tges_GOM = (length(GOM)-1)*dt_GOM;
        GOM(:,1) = 0:dt_GOM:tges_GOM;
    end
    % Zeitverschiebung
%     [idxTest] = FindIdx(GOM,tshift_GOM);
    for idx=1:length(GOM)
        if GOM(idx,1) >= tshift_GOM
            idxA = idx;
            break
        end
    end

    GOM(:,1) = GOM(:,1) - ones(length(GOM),1)*tshift_GOM;
    if tshift_GOM ~=0
        GOM = GOM(idxA:end,:); % timeshift Start
    end

    % GOM_Beschl = data_GOM(:,[1,2:4]);
    GOM_Ort    = GOM(:,[1,2:4]);

    if AS_Ordner =="GOM"
        GOM_a2     = GOM(:,[1,5:7]);

        [GOM_Ort] = Saeubern(GOM_Ort);
        [GOM_a2] = Saeubern(GOM_a2);
        % X-Y-Z-Koordinatenrichtungen anpassen
        [GOM_Ort] = Koordinatenkorrektur(GOM_Ort,AS_Ordner);
        [GOM_a2] = Koordinatenkorrektur(GOM_a2,AS_Ordner);
        % Ableiten nicht nötig, da Beschleunigungsdaten gegeben
        [GOM_IOrt] = Interpolieren(GOM_Ort,dt);
        [GOM_Ia] = Interpolieren(GOM_a2,dt);
    elseif AS_Ordner =="Emily"
        [GOM_Ort] = Saeubern(GOM_Ort);
        % X-Y-Z-Koordinatenrichtungen anpassen
        [GOM_Ort] = Koordinatenkorrektur(GOM_Ort,AS_Ordner);
        [GOM_v] = Ableiten(GOM_Ort);
        [GOM_a] = Ableiten(GOM_v);
    
        [GOM_IOrt] = Interpolieren(GOM_Ort,dt);
        [GOM_Iv] = Interpolieren(GOM_v,dt);
        [GOM_Ia] = Interpolieren(GOM_a,dt);
    end
   

    if GOM_Einbinden == 1
        A(1:length(GOM_IOrt),[14,17,20]) = GOM_IOrt(:,2:4);
        A(1:length(GOM_Iv),[15,18,21]) = GOM_Iv(:,2:4);
        A(1:length(GOM_Ia),[16,19,22]) = GOM_Ia(:,2:4);
    end
    end % Flag_Break_AS
end
%===============================================
%=================== Sensor ====================
%===============================================
if Ermitteln==1
    if Update_S_Daten==1
        [S_Name,Sensor_Pfad]=uigetfile('*.csv','Wähle Sensordaten aus einem Sensorprofilordner aus','C:\Users\Daniel Meyer\sciebo\Projektarbeit\Pendelmodell zur Fluessigkeitssymulation\Messungen\Oberflächenmesseung\Tiefensensor\2. Versuche 14.06\'); % default Ordner ausgewählt
        Update_S_Daten=0;
    end
    Sensor_link=strcat(Sensor_Pfad,S_Name);
    % Auswahl der Sensorart für spätere Berechnung
    S_Tiefensensor=contains(Sensor_link,'Tiefensensor');
    S_Ultraschallsensor = contains(Sensor_link,'Ultraschallsensor');
    if S_Tiefensensor ==1
        S_Ordner = 'Tiefensensor';
    elseif S_Ultraschallsensor==1
        S_Ordner = 'Ultraschallsensor';
    else
        S_Ordner ='';
        flag_break_OS =1;
    end
    if flag_break_OS ==1
        disp('Kein Sensorpfrofilordner ausgewählt oder Vorgang abgebrochen')
    else

        data_S_blank = readmatrix(Sensor_link);

        data_S(:,1:3) = data_S_blank(:,[1,3,2]);%*0.001  % [mm] in [m]

        [S_data] = Saeubern(data_S); % Entfernt Zeilen mit Inhalt 0
        [idxA] =FindIdx(S_data,tshift_SA);
        [idxE] =FindIdx(S_data,tshift_SE);

        for Wdhl =1:Wdhl_S
            %[S_data] = Glaetten(S_data,mw_S);
        end

        % Sensormesstiefen kallibrieren bis zu dem Index der Zeitverschiebung
        [S_K] = Kalibrieren_S_T(S_data,idxA);

        % Zeitverschiebung
        S_K(:,1) = S_K(:,1) - ones(length(S_K),1)*tshift_SA;
        if tshift_SE ==0
            S_ts = S_K(idxA:end,:); % timeshift Start
        else
            S_ts = S_K(idxA:idxE,:); % timeshift Start
        end
        [SpaltenName] = W_Name(S);
        % Sensordaten  auswerten
        [S_ts] = SensorWinkel(S_ts,ds); % Positiver Winkel bei Ausschlag in negativer Koordinatenrichtung
        % Vergleich Winkel durch Ebene bzw durch trigonometrie erzeugt
        % figure
        % plot(S_ts(:,1),S_ts(:,5)...,S_ts(:,1),S_ts(:,6)...
        %     ,S_ts(:,1),S_ts(:,8)...,S_ts(:,1),S_ts(:,9)
        %     )

        % Sensorwinkel zur Simulation auf A übertragen
        % 3. "leerer" Winkel wird nur interpoliert, damit die Interpolationsfunktion 3 Argumente
        % bekommt

        if S_Ordner =="Ultraschallsensor"
%             [S_WINT] = Interpolieren(S_ts(:,[1,5:7]),dt);
            S_W=S_ts(:,[1,5:7]);
        end
        if S_Ordner =="Tiefensensor"
%             [S_WINT] = Interpolieren(S_ts(:,[1,2:4]),dt);
            S_W=S_ts(:,[1,2:4]);
        end
        [S_WINT] = Interpolieren(S_W,dt);
        A(1:length(S_WINT),8:9) = S_WINT(:,2:3);

        if GOM_Einbinden ==0 % Bei GOM_Einbinden wird Bewegungszeit vorgegeben
            Bewegungszeit =0;
        end
        Ampl_S = tshift_Bew+Bewegungszeit;
        if tshift_Bew ~= 0
            [idxAmplS]=FindIdx(S_WINT,Ampl_S);
        else
            idxAmplS = 1;
        end



        % ====== erste Ansicht Winkel ========
        % Sensorwinkel anzeigen
        while CodeFalten == 1
            Sensor_Winkel=figure('Name',string('Sensor_Winkel_Übersicht: '+string(S_Name)),'NumberTitle','off','Units','normalized');
            axSWtheta = subplot(2,1,1);
            lim_theta = max(abs(min(S_W(:,2))),max(S_W(:,2)));
            lim_phi = max(abs(min(S_W(:,3))),max(S_W(:,3)));
            lim_Winkel=max(lim_theta,lim_phi)*1.1;
            plot(axSWtheta,S_W(:,1),S_W(:,2),'b',S_W(:,1),S_W(:,2),'r.'...
                ,'MarkerSize',3.5,'LineWidth',0.3);
            axis(axSWtheta,[0 inf -lim_Winkel lim_Winkel]) %[0 inf -lim_theta lim_theta]
            title(axSWtheta,'Sensorwinkel');
            ylabel(axSWtheta,'Winkel [ °]');
            xlabel(axSWtheta,'t [s]');
            legend(axSWtheta,'Theta','Datenpunkt');
            grid(axSWtheta,'minor');
            hold on
            axSWphi = subplot(2,1,2);
            plot(axSWphi,S_W(:,1),S_W(:,3),'r',S_W(:,1),S_W(:,3),'b.'...
                ,'MarkerSize',3.5,'LineWidth',0.3);
            axis(axSWphi,[0 inf -lim_Winkel lim_Winkel]) %[0 inf -lim_phi lim_phi]
            ylabel(axSWphi,'Winkel [ °]');
            xlabel(axSWphi,'t [s]');
            legend(axSWphi,'Phi','Datenpunkt');
            grid(axSWphi,'minor');
            if tshift_Bew ~=0
                patch(axSWtheta,[tshift_Bew,tshift_Bew+Bewegungszeit...
                    ,tshift_Bew+Bewegungszeit,tshift_Bew]...
                    ,[-lim_Winkel -lim_Winkel lim_Winkel lim_Winkel],[0.9290 0.6940 0.1250] ...[-lim_theta -lim_theta lim_theta lim_theta]
                    ,FaceAlpha = 0.3, EdgeAlpha = 0)
                patch(axSWphi,[tshift_Bew,tshift_Bew+Bewegungszeit...
                    ,tshift_Bew+Bewegungszeit,tshift_Bew]...
                    ,[-lim_Winkel -lim_Winkel lim_Winkel lim_Winkel],[0.9290 0.6940 0.1250] ...[-lim_phi -lim_phi lim_phi lim_phi]
                    ,FaceAlpha = 0.3, EdgeAlpha = 0)
                legend(axSWtheta,'Theta','Datenpunkt','Bewegungszeit' ...
                    );
                legend(axSWphi,'Phi','Datenpunkt','Bewegungszeit' ...
                    );
            end
            Sensor_Winkel.OuterPosition(1:4) = [0.5 0.1 0.5 0.8];
        break
        end% =====================ende erste Ansicht ==============


        % Auswertung durch FFT, Abbklingkostanten fiiting und PT2 fitting
        while CodeFalten == 1
            if tshift_SA == 0 && tshift_Bew == 0
                disp('Kein Zeitabschnitt zur Sensorauswertung ausgewählt');
            else
                %     tshift_SA ~= 0 || tshift_Bew ~= 0 % Auswertung und Erstellung Fourier/PT2 Fitting und Kombifenster nur dann
                % ============= Fast Fourier Transformation ================

                % Use Fourier transforms to find the frequency components of a signal buried in noise.
                % Specify the parameters of a signal with a sampling frequency of 1 kHz and a signal duration of 1.5 seconds.

                Fs = 1/dt;            % Sampling frequency
                T = 1/Fs;             % Sampling period
                L_fft = 2*floor(length(S_WINT(idxAmplS:end,S))/2); % Length of signal 2*floor(X/2) auf gerade Zahl runden
                % Compute the Fourier transform of the signal.
                Y = fft(S_WINT(idxAmplS:end,S)); %S_WINT(:,2) %S_ts(:,5)
                % Compute the two-sided spectrum P2. Then compute the single-sided
                % spectrum P1 based on P2 and the even-valued signal length L.
                P2 = abs(Y/L_fft);
                P1 = P2(1:L_fft/2+1);
                P1(2:end-1) = 2*P1(2:end-1);
                % Define the frequency domain f and plot the single-sided amplitude
                % spectrum P1. The amplitudes are not exactly at 0.7 and 1, as
                % expected, because of the added noise. On average, longer signals
                % produce better frequency approximations.

                f = Fs*(0:(L_fft/2))/L_fft;
                Fourier = [f',P1];
                [FFT_Value, FFT_Hz] = max(Fourier(:),[],'all');
                %         MaxFFT = max(Fourier);
                %Lokale Maxima durch "1" angezeigt
                MaxFFT = islocalmax(Fourier,'MaxNumExtrema',5);
                %         Max_FFT=max
                MaxFFT = MaxFFT(:,2);
                %         idxFFT = find(P1==MaxFFT(2));
                % Indizes der lokalen maxima ermitteln
                idxMax_L= find(MaxFFT);
                % Werte an den Indizes ermitteln
                F=Fourier(idxMax_L,2);
                % Maximum der Werte ermitteln
                f_V=max(Fourier(idxMax_L,2));
                % idx des Maximalen wertes
                idxMax=find(Fourier(:,2)==f_V);
                % Wenn der Index kleiner als 5 ist spricht dass dafür das es einen
                % Frequenzpeak bei 1/tges gibt, der die auswertung verzehrt
                while idxMax<5
                    F(find(idxMax))=0;
                    f_v=max(F);
                    idxMax=find(Fourier(:,2)==f_v);
                end
                f_d=Fourier(idxMax,1);
                if f_d <=1.5
                    disp('Frequenz überprüfen')
                end
                % Frequenz ausgeben ====================
                % Frequenzspektrum
                Frequenz=figure('Name',string('Frequenzspektrum: '+string(S_Name)),'NumberTitle','off','Units','normalized');
                axF = axes;
                plot(axF,f,P1,'b',f,P1,'r.');
                title('Single-Sided Amplitude Spectrum of: "'+ string(S_Name) +'"','Interpreter','none')
                xlabel('f (Hz)')
                ylabel('|P1(f)|')
                %annotation(Frequenz,'textbox',[0.3 0.75 0.1 0.1],'String','Frequenzpeak: ' + string(f_d) + ' [Hz]');
                axis(axF,[0 10 0 f_d*1.1])
                grid(axF,'minor');
                % Frequenz ausgeben Ende===============

                % dedektieren der Hüllkurve
                Amp = S_WINT(idxAmplS:end,S);
                % e_time = S_WINT(idxAmplS:end,1);
                Amp_time = (0:dt:(length(S_WINT)-idxAmplS)*dt)';
                [up,lo] = envelope(Amp,25,'analytic');

                % ============ Abklingkonstante fitting =============================

                
                clear fit
                [xData, yData] = prepareCurveData( Amp_time, up );
                % Set up fittype and options.
                ft_up = fittype( 'C*exp(-d*x)', 'independent', 'x', 'dependent', 'y' );
                opts_up = fitoptions( 'Method', 'NonlinearLeastSquares' );
                opts_up.Display = 'Off';
                % opts_up.StartPoint = [0.913375856139019 0.63235924622541];
                opts_up.Lower = 0;


                % Fit model to data.
                [fitresult_up, gof_up] = fit( xData, yData, ft_up, opts_up);
                C_up = fitresult_up.C;
                ak_up= fitresult_up.d;

                % =============================
                [xData, yData] = prepareCurveData( Amp_time, lo );

                % Set up fittype and options.
                ft_lo = fittype( '-C*exp(-d*x)', 'independent', 'x', 'dependent', 'y' );
                opts_lo = fitoptions( 'Method', 'NonlinearLeastSquares' );
                opts_lo.Display = 'Off';
                opts_lo.Lower = 0;

                % Fit model to data.
                [fitresult_lo, gof_lo] = fit( xData, yData, ft_lo, opts_lo );
                C_lo = fitresult_lo.C;
                ak_lo= fitresult_lo.d;

                % Vorläufige Parameter für die Amplitudenfunktion
                C_amp_vl = (C_up+C_lo)/2;
                ak_amp_vl = (ak_up+ak_lo)/2;
                if ak_amp_vl<=0
                    warning('Vorläufig ermittelte Abklingkonstante ist negativ: '+string(ak_amp_vl))
                    disp('Abklingkonstant gesetzt zu: 0')
                    ak_amp_vl=0;
                end
                % Daraus errechnen der vorläufigen Dämpfung zeta_err
                w_d=2*pi*f_d;
                w_0 = sqrt(w_d^2+ak_amp_vl^2);
                L_err = g/w_0^2;
                zeta_err=ak_amp_vl/w_0;
                % d_err = ak_amp*2*M40

                % ======== PT2 Fitting =================
                


                [xData, yData] = prepareCurveData( Amp_time, Amp );
                % Set up fittype and options.
                % t als Variable zur Verschiebung des Zeitvektors um einen
                % optimalen Startpunkt zu erreichen
                ft = fittype( 'C/sqrt(1-zeta.^2).*exp(-zeta.*w.*(x-t)).*sin(w.*sqrt(1-zeta.^2).*(x-t))', 'independent', 'x', 'dependent', 'y' );
                opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
                opts.DiffMaxChange = 10;
                opts.Display = 'Off';
                opts.StartPoint = [C_amp_vl 0 w_d zeta_err];
                opts.Lower = [C_amp_vl -1 w_d*0.95 zeta_err*0.5];
                opts.Upper = [90 1 w_d*1.1 zeta_err*1.3];

                % Fit model to data.
                [fitresult, gof] = fit( xData, yData, ft, opts )
                % rsquare=gof.rsquare;
                % Werte aus Fitting Funktion
                C_f = fitresult.C;
                % C_f = 35;

                w_f = fitresult.w; %13.26
                f_f = w_f/(2*pi);
                L_f = g/w_0^2;
                zeta_f = fitresult.zeta;
                % zeta_f = 0.004;
                d_f = zeta_f*2*M*w_0;
                Amp_time_korr=fitresult.t;
                

                % PT2 = C/sqrt(1-zeta.^2).*exp(-zeta.*w.*x).*sin(w.*sqrt(1-zeta.^2).*x);
                PT2 = C_f/sqrt(1-zeta_f.^2).*exp(-zeta_f.*w_f.*Amp_time).*sin(w_f.*sqrt(1-zeta_f.^2).*Amp_time);

                % ================== Curve fitting ende  ============================

                Ampl_time = Amp_time+Amp_time_korr+Ampl_S;
                % LogDek voläufig
                Ampl_up_vl = C_amp_vl*exp(-ak_amp_vl.*Amp_time);
                Ampl_lo_vl = -C_amp_vl*exp(-ak_amp_vl.*Amp_time);
                % Funktion für Logarithmisches Dekrement
                Ampl_up_f = C_f*exp(-zeta_f*w_0.*Amp_time);
                Ampl_lo_f = -C_f*exp(-zeta_f*w_0.*Amp_time);

                % ==================== Kombi-Ansicht ============================
                while CodeFalten == 1
                    Kombi=figure('Name',string('Kombi-Fenster: '+string(S_Name)),'NumberTitle','off','Units','normalized');
                    % Frequenzspektrum
                    if S_Ordner=="Ultraschallsensor"
                        ax11=subplot(2,2,[1,3]);
                    end
                    if S_Ordner=="Tiefensensor"
                        ax11=subplot(3,1,3);
                    end

                    plot(ax11,f,P1,'b',f,P1,'r.');
                    title('Single-Sided Amplitude Spectrum of: "'+ string(S_Name) +'"','Interpreter','none')
                    xlabel('f (Hz)')
                    ylabel('|P1(f)|')
                    annotation(Kombi,'textbox',[0.3 0.15 0.1 0.1],'String','Frequenzpeak: ' + string(f_d) + ' [Hz]');
                    axis(ax11,[0 20 -inf inf])

                    axSW=subplot(2,2,2);
                    if S_Ordner =="Ultraschallsensor"
                        % axSW = axes;
                        ylabel(axSW,'Winkel [ °]');
                        xlabel(axSW,'t [s]');
                        % axis manual
                        axis(axSW, [0 inf -inf inf]);
                        % plot(axSW,data_S(:,1),data_S(:,5),data_S(:,1),data_S(:,6))
                        hold on
                        plot(axSW,S_WINT(:,1),S_WINT(:,S))
                        legend(axSW ...
                            ... ,'Theta','Phi'...
                            ,'Theta'...
                            ...,'Phi'...
                            );
                        title(axSW,'Sensorwinkel');
                    end
                    % Winkelverlauf
                    if S_Ordner=="Tiefensensor"
                        axSWtheta = subplot(3,1,1);
                        plot(axSWtheta,S_W(:,1),S_W(:,S),'b',S_W(:,1),S_W(:,S),'r.');
                        title(axSWtheta,'Gemessener Winkelverlauf');
                        legend(axSWtheta,SpaltenName);
                        ylabel(axSWtheta,'Winkel [ °]');
                        xlabel(axSWtheta,'t [s]');
                        lim_theta = max(abs(min(S_W(:,S))),max(S_W(:,S)));
                        axis(axSWtheta,[0 inf -lim_theta lim_theta]) %max(S_WINT(:,1))
                    end

                    if S_Ordner=="Ultraschallsensor"
                        axS = subplot(2,2,4);
                        hold on
                        plot(axS,S_INT(:,1),S_INT(:,2),S_INT(:,1),S_INT(:,3),S_INT(:,1),S_INT(:,4))
                        legend(axS, ...
                            ...'Z1','Z2','Z3',...
                            'Z1_G','Z2_G',...
                            'Z3_G'...
                            );
                        grid(axS,'minor');
                        title(axS,'Abstand zum Sensor');
                        if tshift_SA ~= 0
                            title(axS,'Höhe des Messpunktes über Normalenfläche');
                        end
                        ylabel(axS,'Abstand [m]');
                        xlabel(axS,'t [s]');
                    end

                    % PT2 Fit
                    if S_Ordner=="Tiefensensor"
                        axAmp=subplot(3,1,2);
                        plot(axAmp,S_W(:,1),S_W(:,S),'r.',S_W(:,1),S_W(:,S),'cyan-'...
                            ,'MarkerSize',4,'LineWidth',0.3);
                        hold on % Aufteilen für unterschiedliche LineWidth/MarkerSize
                        plot(axAmp,Ampl_time,PT2,'b'...
                            ,'MarkerSize',4,'LineWidth',0.3);
                        plot(axAmp,Ampl_time,Ampl_up_f,'g'...,Ampl_time,Ampl_up_vl,'c',Ampl_time,up,'r'...
                            ,Ampl_time,Ampl_lo_f,'g'...,Ampl_time,Ampl_lo_vl,'c',Ampl_time,lo,'r'...
                            ,'LineWidth',0.5);

                        title(axAmp,'PT2 Antwort')
                        ylabel(axAmp,'Winkel [ °]');
                        xlabel(axAmp,'t [s]');
                        if M==0
                            subtitle(axAmp,{'Pendellänge: '+string(round(L_f,4))+ ' [m]'+...
                               ' und Abklingkonstante: '+ string(round(zeta_f,4))}...
                                );
                        else
%                             subtitle(axAmp,{'Pendellänge: '+string(round(L_f,4))+ ' [m]'+...
%                                 ' und Dämpfungskonstante: '+ string(round(d_f,4)) + ' [N/(m/s)]'}...
%                                 );
                        end
                        legend(axAmp,'Daten','Linear verbunden','PT2 Antwort'...
                            ,'Amplitudenfunktion'...
                            ...,'gemittelte Amplitudenfunktion'...
                            ...,'envelope' ...
                            )
                        grid(axAmp,'minor')
                        %annotation(Kombi,"textbox",[0.82 0.55 0.1 0.1],'String',{'PT2-Fit','rsquare: ' + string(round(gof.rsquare,3))});
                    end

                    %     f2.OuterPosition(1:4) = [0.1 0.1 0.8 0.8];
                break
                end% =================== Ende Kombi-Ansicht ===========================


                %         % Frequenzspektrum
                %         Frequenz=figure('Name',string('Frequenzspektrum: '+string(S_Name)),'NumberTitle','off','Units','normalized');
                %         axF = axes;
                %         plot(axF,f,P1,'b',f,P1,'r.');
                %         title('Single-Sided Amplitude Spectrum of: "'+ string(S_Name) +'"','Interpreter','none')
                %         xlabel('f (Hz)')
                %         ylabel('|P1(f)|')
                %         annotation(Frequenz,'textbox',[0.3 0.75 0.1 0.1],'String','Frequenzpeak: ' + string(f_d) + ' [Hz]');

                % PT2 Impulsantwort mit errechneten Pendelparametern
                while CodeFalten == 1
                    fit=figure('Name',string('PT2-Impulsntwort '+string(S_Name)),'NumberTitle','off','Units','normalized');
                    axAmp=axes;
                    plot(axAmp,S_WINT(:,1),S_WINT(:,S),'b.');%,S_WINT(:,1),S_WINT(:,S),'b-');
                    hold on % Aufteilen für unterschiedliche LineWidth/MarkerSize
                    plot(axAmp,Ampl_time,PT2,'b'...
                        ,'MarkerSize',3.5,'LineWidth',0.3);
                    plot(axAmp,Ampl_time,Ampl_up_f,'g','LineWidth',1)
                    r=[1 0 0];
                    or=[0.8500 0.3250 0.0980];
                    haxAmp=plot(axAmp,Ampl_time,up,Ampl_time,Ampl_up_vl...'g'... r und c
                        ,Ampl_time,lo,Ampl_time,Ampl_lo_vl...
                        ,'LineWidth',1);
                    set(haxAmp(2),'Color',[0 1 0])
                    set(haxAmp(4),'Color',[0 1 0])
                    set(haxAmp(1),'Color',[1 0 0])
                    set(haxAmp(3),'Color',[1 0 0])
                    plot(axAmp,Ampl_time,Ampl_lo_f,'g','LineWidth',1)
                    legend(axAmp,'Daten','Linear verbunden'...
                        ...,'PT2 Antwort'...
                        ...,'Amplitudenfunktion'...
                        ,'Hüllfunktion'...
                        ,'vorläufige Amplitudenfunktion'...
                        )
%                     annotation(fit,"textbox",[0.1 0.9 0.1 0.1],'String',{'rsquare: ' + string(round(gof.rsquare,3))});
                    title(axAmp,'Hüllkontour der abklingenden Schwingung') % 'PT2 Antwort'
                    ylabel(axAmp,'Winkel [ °]');
                    xlabel(axAmp,'t [s]');
                    if M==0
                        subtitle(axAmp,{'Pendellänge: '+string(round(L_f,4))+ ' [m]'+...
                            ' und Abklingkonstante: '+ string(round(zeta_f,4))}...
                            );
                    else
%                         subtitle(axAmp,{'Pendellänge: '+string(round(L_f,4))+ ' [m]'+...
%                             ' und Dämpfungskonstante: '+ string(round(d_f,4)) + ' [N/(m/s)]'}...
%                             );
                    end
                break
                end % Ende PT2 Impulsantwort
            end
        
        break
        end % ====== Ende Auswertung FT, Abbklingkostanten fiiting und PT2 fitting=======
    end % flag_break_OS vom Fehlerabfang beim Daten auswählen
end
%===============================================
%================= Berechnung ==================
%===============================================
while CodeFalten==1
    %============== lineare Strecke errechnete Größen==========================
    while CodeFalten == 1
        if PivotLinear == 1
            taX = 2*(vmaxX/amaxX);  % Dauer der Beschleunigung
            taY = 2*(vmaxY/amaxY);
            taZ = 2*(vmaxZ/amaxZ);
            s12X = 0.5*vmaxX*taX;   % Wegfunktion für 2 pi vereinfacht
            s12Y = 0.5*vmaxY*taY;
            s12Z = 0.5*vmaxZ*taZ;
            % amax kann zu niedrig gewählt sein, sodass der Weg in der
            % Beschleunigungsphase auf vmax bereits überschritten wir, da ta abhängig
            % ist über vmax, dann wird das benötigte kleinst mögliche a ermittelt
            % und damit ta neu berrechnet
            if taX2 ==0
                if (abs(sgesX)<abs(2*s12X))
                    amaxX = vmaxX^2/(sgesX/2);
                    taX = 2*(vmaxX/amaxX);
                    s12X = 0.5*vmaxX*taX;
                    disp('vmax konnte nicht erreicht werden. amaxX wurde zu niedrig gewählt. Kleinst mögliche amamX ermittelt zu: '+string(amaxX))
                end
            end
            if taY2 ==0
                if (abs(sgesY)<abs(2*s12Y))
                    amaxY= (vmaxY^2)/(sgesY/2);
                    taY = 2*(vmaxY/amaxY);
                    s12Y = 0.5*vmaxY*taY;
                    disp('vmax konnte nicht erreicht werden. amaxY wurde zu niedrig gewählt. Kleinst mögliche amamX ermittelt zu: '+string(amaxY))
                end
            end
            if taZ2 ==0
                if (abs(sgesZ)<abs(2*s12Z))
                    amaxZ= vmaxZ^2/(sgesZ/2);
                    taZ = 2*(vmaxZ/amaxZ);
                    s12Z = 0.5*vmaxZ*taZ;
                    disp('vmax konnte nicht erreicht werden. amaxZ wurde zu niedrig gewählt. Kleinst mögliche amamX ermittelt zu: '+string(amaxZ))
                end
            end
            % ta kann zu hoch gewählt sein, sodass der Weg in der
            % Beschleunigungsphase auf vmax bereits überschritten wir, da amax abhängig
            % ist über vmax, dann wird das benötigte größt mögliche ta ermittelt
            % und damit amax neu berrechnet
            if taX2 ~= 0
                taX = taX2;
                amaxX = 2*(vmaxX/taX);
                s12X = 0.5*vmaxX*taX;
                if (abs(sgesX)<abs(2*s12X))
                    taX = 2*((sgesX/2)/vmaxX);
                    amaX = 2*(vmaxX/taX);
                    s12X = 0.5*vmaxX*taX;
                    disp('vmax konnte nicht erreicht werden. ta wurde zu hoch gewählt. Größt möglicher taX Wert ermittelt zu: ' +string(taX))
                end
            end
            if taY2 ~= 0
                taY = taY2;
                amaxY = 2*(vmaxY/taY);
                s12Y = 0.5*vmaxY*taY;
                if (abs(sgesY)<abs(2*s12Y))
                    taY = 2*((sgesY/2)/vmaxY);
                    amaY = 2*(vmaxY/taY);
                    s12Y = 0.5*vmaxY*taY;
                    disp('vmax konnte nicht erreicht werden. ta wurde zu hoch gewählt. Größt möglicher taY Wert ermittelt zu: ' +string(taY))
                end
            end
            if taZ2 ~= 0
                taZ = taZ2;
                amaxZ = 2*(vmaxZ/taZ);
                s12Z = 0.5*vmaxZ*taZ;
                if (abs(sgesZ)<abs(2*s12Z))
                    taZ = 2*((sgesZ/2)/vmaxZ);
                    amaZ = 2*(vmaxZ/taZ);
                    s12Z = 0.5*vmaxZ*taZ;
                    disp('vmax konnte nicht erreicht werden. ta wurde zu hoch gewählt. Gößt möglicher taZ Wert ermittelt zu: ' +string(taZ))
                end
            end
            t2X = t1X+taX;     % Zeitpunkt beginn Abbremsen
            t3X = t2X+(sgesX-2*s12X)/vmaxX;
            tmaxX= t3X+taX;
            % t1Y = tmaxX; % Überschrieben, damit 90° Weg direkt im Anschluss ist
            t2Y = t1Y+taY;     % Zeitpunkt beginn Abbremsen
            t3Y = t2Y+(sgesY-2*s12Y)/vmaxY;
            tmaxY = t3Y+taY;
            t2Z = t1Z+taZ;     % Zeitpunkt beginn Abbremsen
            t3Z = t2Z+(sgesZ-2*s12Z)/vmaxZ;
            tmaxZ= t3Z+taZ;
        end
        break
    end
    
    %==================Kreisbahn errechnete Größen=============================
    while CodeFalten == 1
        if PivotCircular ==1
            OX = ([1; 0; 0]); % Gerade für Winkel
            VS = PS-PM; %Vektor zum Startpunkt
            VR = PS-PM; % Vektor für Radiusgröße
            r=norm(VR);
            aBogenRAD = aBogen*2*pi/360;
            n=n*r;
            % Winkel zwischen X normal und gedrehtem System
            as = real(acos(norm(dot(OX,VS))/(norm(OX)*norm(VS)))); % (OX(1)*VS(1)+OX(2)*VS(2)+OX(3)*VS(3))
            % Winkel wird von X Richtung aus gemessen, wenn SY PMY unterschreitet, wird
            % die Vektorielle Winkelberechnung nur den Gegenwinkel ausgeben
            if PS(2)<PM(2), as = 2*pi-as; end
            sgesB = aBogen*2*pi*r/360;
            amaxzp=vmaxB^2/r; % vB^2/rB  % Zentripetalbeschleunigung
            wmaxB=vmaxB/r;
            taB = 2*(vmaxB/amaxB);  % Dauer der Beschleunigung
            s12B = 0.5*vmaxB*taB;
            if (abs(sgesB)<abs(2*s12B))
                amaxB = vmaxB^2/(sgesB/2);
                taB = 2*(vmaxB/amaxB);
                s12B = 0.5*vmaxB*taB;
            end
            t2B=t1B+taB;
            t3B = t2B+(sgesB-2*s12B)/vmaxB;
            tmaxB= t3B+taB;
        end
        break
    end
    % =============== Iterative Berechnung der Ergebnismatrix =================
    j=1;
    while j<=tges/dt
        % Zeit
        %     A(j+1,1)= A(j,1)+dt; % In A-Erzeugung festgelegt

        %     % Anfangsimpuls der Beschleunigung X System
        %         if (n*eta<=A(j+1,1))&& (A(j+1,1)<(n+1)*eta) %bis eta
        %             A(j+1,16) = 1/eta;  %A(1,19); %1/eta;
        %         elseif ((n+2)*eta<=A(j+1,1))&& (A(j+1,1)<5.3+eta) %bis eta
        %                 A(j+1,16) =0;   %-A(1,19); %-1/eta;
        %             else
        %                 A(j+1,16)=0;
        %         end

        %     % Zufallsvariable für künstliches Rauschen
        %     rng('default');
        %     mu = A(j+1,16);
        %     sigma = abs(A(j+1,16) * 0.1);
        %     A(j+1,16) = random('Normal',mu,sigma);
        %  Bei Zeiten für die Abschnitte vorgegeben, nur Beschleunigung betrachten
        %  und Geschwindigkeit und Weg durch einfache numerische Integration.
        % bei betrachtung von endweg, amax und vmax nicht möglich, da man bereits
        % vorher für die "Zeitplanung" den Wegfortschritt braucht

        % Berechnung Linear- und Kreisbahn
        while CodeFalten ==1
            %================== Bewegungsgleichungen Linear =======================
            if PivotLinear == 1
                % Ausgliederung in Funktionen verlängert die Berechnungszeit um ein
                % vielfaches
                %========================== X ===================================
                if (A(j+1,1)<t1X)
                    A(j+1,16) = 0;
                    A(j+1,15) = 0;
                    A(j+1,14) = 0;
                elseif (t1X<=A(j+1,1)) && (A(j+1,1)<t2X)   % Beschleunigung
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((A(j+1,1)-t1X)/taX);
                    % Beschleunigung
                    A(j+1,16) = ((-cos(p)+1)/taX*vmaxX);
                    % Geschwindigkeit
                    A(j+1,15) = (-sin(p)+p)/(2*pi)*vmaxX;
                    % Weg
                    A(j+1,14) = ((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxX*(taX/(2*pi));
                elseif ((t2X)<=A(j+1,1)) && (A(j,1)<t3X) % Gleichbleibend
                    A(j+1,16) = 0;
                    A(j+1,15) = vmaxX;
                    A(j+1,14) = s12X + (sgesX-2*s12X)*((A(j+1,1)-t2X)/(t3X-t2X));
                elseif (t3X<=A(j+1,1)) && A(j+1,1)<(tmaxX) % Abbremsen
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((tmaxX-(A(j+1,1)))/taX);
                    % Beschleunigung
                    A(j+1,16) = -((-cos(p)+1)/taX*vmaxX);
                    % Geschwindigkeit
                    A(j+1,15) = (-sin(p)+p)/(2*pi)*vmaxX;
                    % Weg
                    A(j+1,14) = sgesX-((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxX*(taX/(2*pi));
                else
                    A(j+1,16) = 0;
                    A(j+1,15) = 0;
                    A(j+1,14) = sgesX;
                end
                %========================== Y ===================================
                if (A(j+1,1)<t1Y)
                    A(j+1,19) = 0;
                    A(j+1,18) = 0;
                    A(j+1,17) = 0;
                elseif (t1Y<=A(j+1,1)) && (A(j+1,1)<t2Y)   % Beschleunigung
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((A(j+1,1)-t1Y)/taY);
                    % Beschleunigung
                    A(j+1,19) = ((-cos(p)+1)/taY*vmaxY);
                    % Geschwindigkeit
                    A(j+1,18) = (-sin(p)+p)/(2*pi)*vmaxY;
                    % Weg
                    A(j+1,17) = ((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxY*(taY/(2*pi));
                elseif ((t2Y)<=A(j+1,1)) && (A(j,1)<t3Y) % Gleichbleibend
                    A(j+1,19) = 0;
                    A(j+1,18) = vmaxY;
                    A(j+1,17) = s12Y + (sgesY-2*s12Y)*((A(j+1,1)-t2Y)/(t3Y-t2Y));
                elseif (t3Y<=A(j+1,1)) && A(j+1,1)<(tmaxY) % Abbremsen
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((tmaxY-(A(j+1,1)))/taY);
                    % Beschleunigung
                    A(j+1,19) = -((-cos(p)+1)/taY*vmaxY);
                    % Geschwindigkeit
                    A(j+1,18) = (-sin(p)+p)/(2*pi)*vmaxY;
                    % Weg
                    A(j+1,17) = sgesY-((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxY*(taY/(2*pi));
                else
                    A(j+1,19) = 0;
                    A(j+1,18) = 0;
                    A(j+1,17) = sgesY;
                end
                %============================ Z ===================================
                if (A(j+1,1)<t1Z)
                    A(j+1,22) = 0;
                    A(j+1,21) = 0;
                    A(j+1,20) = 0;
                elseif (t1Z<=A(j+1,1)) && (A(j+1,1)<t2Z)   % Beschleunigung
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((A(j+1,1)-t1Z)/taZ);
                    % Beschleunigung
                    A(j+1,22) = ((-cos(p)+1)/taZ*vmaxZ);
                    % Geschwindigkeit
                    A(j+1,21) = (-sin(p)+p)/(2*pi)*vmaxZ;
                    % Weg
                    A(j+1,20) = ((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxZ*(taZ/(2*pi));
                elseif ((t2Z)<=A(j+1,1)) && (A(j,1)<t3Z) % Gleichbleibend
                    A(j+1,22) = 0;
                    A(j+1,21) = vmaxZ;
                    A(j+1,20) = s12Z + (sgesZ-2*s12Z)*((A(j+1,1)-t2Z)/(t3Z-t2Z));
                elseif (t3Z<=A(j+1,1)) && A(j+1,1)<(tmaxZ) % Abbremsen
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((tmaxZ-(A(j+1,1)))/taZ);
                    % Beschleunigung
                    A(j+1,22) = -((-cos(p)+1)/taZ*vmaxZ);
                    % Geschwindigkeit
                    A(j+1,21) = (-sin(p)+p)/(2*pi)*vmaxZ;
                    % Weg
                    A(j+1,20) = sgesZ-((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxZ*(taZ/(2*pi));
                else
                    A(j+1,22) = 0;
                    A(j+1,21) = 0;
                    A(j+1,20) = sgesZ;
                end
            end
            % ============= Bewegungsgleichungen Kreisbewegung =================
            % Ansatzfunktion für die Bahngeschwindigkeit !!!
            % Beschleunigungsvektor X/Y aus zugehöriger Zentripetalbeschleunigung
            % Berechnung im System um M(0/0/0) mit alpha von X nach Y
            % anschließend Verschiebung und Rotation ins neue System
            if PivotCircular == 1
                if (A(j+1,1)<t1B) %Startwerte auch in Update für erste Zeile berrechnet
                    A(j+1,16) = 0; % Beschleunigung
                    A(j+1,19) = 0;
                    A(j+1,15) = 0; % Geschwindigkeit
                    A(j+1,18) = 0;
                    [A(j+1,14), A(j+1,17),A(j+1,20)] = KoordTrans(r,0,as,PM);
                elseif (t1B<=A(j+1,1)) && (A(j+1,1)<t2B)   % Beschleunigung
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    prozent = ((A(j+1,1)-t1B)/taB); % Prozent 0-100% des Fortschrittes
                    p = 2*pi*prozent;
                    % Bahngeschwindigkeit im steigernden verlauf
                    vB = ((-sin(p)+p)*vmaxB/(2*pi));    % Bahngeschwindigkeit
                    A(j+1,27) = vB;
                    azp = vB^2/r;                       % Zentripetalbeschleunigung
                    A(j+1,28) = azp;
                    wB = vB/r;                          % Winkelgeschwindigkeit ist Eigenkreisfrequenz
                    A(j+1,29) = wB;
                    % Position des Verstrichenen Winkels, Integral vB über Zeit,
                    % zurückgelegter Bogenlänge, a und (v) zu dieser Zeit in
                    % Koordinaten aufteilen#
                    % Weg
                    sB = ((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxB*(taB/(2*pi)); % Strecke Bogenmaß
                    aB = sB/r; % Winkel in rad
                    [A(j+1,14), A(j+1,17),A(j+1,20)] = KoordTrans(r,aB,as,PM);
                    Pnow = ([A(j+1,14);A(j+1,17);A(j+1,20)]);  % Z Komponente bleibt erstmal 0
                    Vazp = PM-Pnow;  %Vektor azp
                    Vnorm = Vazp/norm(Vazp); % Richtungsvektor auf Länge "1" skalliert
                    A(j+1,16) = azp*Vazp(1);
                    A(j+1,19) = azp*Vazp(2);
                    A(j+1,22) = azp*Vazp(3);
                elseif ((t2B)<=A(j+1,1)) && (A(j+1,1)<t3B)  % Gleichbleibend
                    vB = vmaxB;    % Bahngeschwindigkeit
                    A(j+1,27) = vB;
                    azp = vB^2/r;                       % Zentripetalbeschleunigung
                    A(j+1,28) = azp;
                    wB = vB/r;                          % Winkelgeschwindigkeit ist Eigenkreisfrequenz
                    A(j+1,29) = wB;
                    % Weg
                    sB = s12B + (sgesB-2*s12B)*((A(j+1,1)-t2B)/(t3B-t2B)); % Strecke Bogenmaß
                    aB = sB/r; % Winkel in rad
                    [A(j+1,14), A(j+1,17),A(j+1,20)] = KoordTrans(r,aB,as,PM);
                    Pnow = ([A(j+1,14);A(j+1,17);A(j+1,20)]);  %Z Komponente bleibt erstmal 0
                    Vazp = PM-Pnow;  %Vektor azp
                    Vnorm = Vazp/norm(Vazp); % Richtungsvektor auf Länge "1" skalliert
                    A(j+1,16) = azp*Vazp(1);
                    A(j+1,19) = azp*Vazp(2);
                    A(j+1,22) = azp*Vazp(3);
                elseif (t3B<=A(j+1,1)) && A(j+1,1)<(tmaxB) % Abbremsen
                    % Zeitfunktion zur skalierung der trigonometrischen Ansatzfunktion
                    p = 2*pi*((tmaxB-(A(j+1,1)))/taB);
                    % Bahngeschwindigkeit im sinkendem verlauf
                    vB = ((-sin(p)+p)*vmaxB/(2*pi));    % Bahngeschwindigkeit
                    A(j+1,27) = vB;
                    azp = vB^2/r;                       % Zentripetalbeschleunigung
                    A(j+1,28) = azp;
                    wB = vB/r;                          % Winkelgeschwindigkeit ist Eigenkreisfrequenz
                    A(j+1,29) = wB;
                    sB = sgesB-((cos(p)+0.5*(p^2)-1)/(2*pi))*vmaxB*(taB/(2*pi)); % Strecke Bogenmaß
                    aB = sB/r; % Winkel in rad
                    [A(j+1,14), A(j+1,17),A(j+1,20)] = KoordTrans(r,aB,as,PM);
                    Pnow = ([A(j+1,14);A(j+1,17);A(j+1,20)]);  %Z Komponente bleibt erstmal 0
                    Vazp = PM-Pnow;  %Vektor azp
                    Vnorm = Vazp/norm(Vazp); % Richtungsvektor auf Länge "1" skalliert
                    A(j+1,16) = azp*Vazp(1);
                    A(j+1,19) = azp*Vazp(2);
                    A(j+1,22) = azp*Vazp(3);
                else
                    A(j+1,16) = 0; % Beschleunigung
                    A(j+1,19) = 0;
                    A(j+1,15) = 0; % Geschwindigkeit
                    A(j+1,18) = 0;
                    [A(j+1,14), A(j+1,17),A(j+1,20)] = KoordTrans(r,aBogenRAD,as,PM);
                end
            end
            break
        end

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
        % Mathematische Funktion
        %     A(j+1,2) = A(1,2)*cos(w*A(j+1,1));
        %     A(j+1,5) = A(1,5)*cos(w*A(j+1,1));
        % PT2 Impulsantwort
        %         A(j+1,2) = A(1,2)/wzl*exp(-zeta*w*A(j+1,1))*cos(w*A(j+1,1)*wzl);
        %         A(j+1,5) = A(1,5)/wzl*exp(-zeta*w*A(j+1,1))*cos(w*A(j+1,1)*wzl);
        % Ausgeschrieben
        % zeta = d/(2*M*w);
        % wzl = sqrt(1-zeta^2); % Wurzelterm der PT impulsantwort
        % Winkel = A/wzl*exp(-zeta*w*t)*sin(w*wzl*t);
        % Winkel = A/sqrt(1-zeta^2)*exp(-zeta*w*t)*sin(w*sqrt(1-zeta^2)*t);
        % Winkel = A/sqrt(1-(d/(2*m*sqrt(g/L))^2)*exp(-(d/(2*m*sqrt(g/L)))*sqrt(g/L)*x)*sin(sqrt(g/L)sqrt(1-(d/(2*m*sqrt(g/L))^2)*x);

        % Folgegrößen errechnen für die erste Zeile aus den Anfangsbedingungen
        % berrechnen
        if j==1
            if PivotCircular == 1
                [A(j,14), A(j,17),A(j,20)] = KoordTrans(r,0,as,PM);
            end

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

            if SIM_Winkel_S == 1 % Überschreiben für Werte aus SIM
                % Für SIM Winkel
                A(j:j+1,23) = cos(A(j:j+1,9)).*sin(A(j:j+1,8));
                A(j:j+1,24) = sin(A(j:j+1,9)).*cos(A(j:j+1,8));
                A(j:j+1,25) = cos(A(j:j+1,9)).*cos(A(j:j+1,8));
                A(j:j+1,26) = sqrt(A(j:j+1,23).^2+A(j:j+1,24).^2+A(j:j+1,25).^2);
                % x_P
                A(j:j+1,11) =- L/A(j:j+1,26) * A(j:j+1,23);
                % y_P
                A(j:j+1,12) =- L/A(j:j+1,26) * A(j:j+1,24);
                % z_P
                A(j:j+1,13) =- L/A(j:j+1,26) * A(j:j+1,25);
            end
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

            if SIM_Winkel_S == 1 % Überschreiben für Werte aus SIM
                % für SIM Winkel
                A(j+1,23) = cos(A(j+1,9))*sin(A(j+1,8));
                A(j+1,24) = sin(A(j+1,9))*cos(A(j+1,8));
                A(j+1,25) = cos(A(j+1,9))*cos(A(j+1,8));
                A(j+1,26) = sqrt(A(j+1,23)^2+A(j+1,24)^2+A(j+1,25)^2);
                % x_P
                A(j+1,11) =- L/A(j+1,26) * A(j+1,23);
                % y_P
                A(j+1,12) =- L/A(j+1,26) * A(j+1,24);
                % z_P
                A(j+1,13) =- L/A(j+1,26) * A(j+1,25);
            end
            %     A(j+1,30) = acosd((A(j+1,13))/L);   %SloshingAngle
            %     A(j+1,30) = asin(A(j+1,25)/A(j+1,26));
        end
        j=j+1; %Nächster Berechnungsschritt
    end
break
end

%===================================================
%====== Erstellen der Ansichtsfenster/Graphen ======
%===================================================
while CodeFalten == 1
    set(0,'Units','normalized')
    get(0,'ScreenSize');
    if Simulieren == 1
        % ============= Winkelamplitudenverlauf erstellen ===================
        Amplitudenverlauf= figure('Name','Simulation Amplitudenverlauf','NumberTitle','off','Units','normalized');
        ax11 = subplot(1,4,[1,2,3]);
        plot(ax11,A(:,1),A(:,2)*360/(2*pi),'red',A(:,1),A(:,5)*360/(2*pi),'blue'...
           ...,Amp_time,PT2...
            ); %theta & phi & Sloshing  und PT2
        hold on
        % plot(ax11,A(:,1),A(:,11),'magenta',A(:,1),A(:,12),'cyan');   %x_p & y_pTheta
        % gemessener Sensorwinkel integrieren
        %plot(ax11,A(:,1),A(:,8),'m',A(:,1),A(:,9),'c'); 
        ylabel(ax11,'Winkel [ ° ]');
        xlabel(ax11,'t [s]');
        title('Simulation Winkelamplitudenverlauf','FontSize', 10)
        grid(ax11,'minor');
        legend(ax11,'Theta - X','Phi - Y'...
            ...,'x_p','y_p'...
            ...,'Theta_S','Phi_S'...
            );
        if SloshingAngle == 1
            plot(ax11,A(:,1),A(:,30),'g');
            legend(ax11,'Theta - X','Phi - Y'...
            ...,'x_p','y_p'...
            ,'Sloshing'...
            ...,'PT2 Antwort'...
            ...,'Theta_S','Phi_S'...
            );
        end

        Amplitudenverlauf.OuterPosition(1:4) = [0 0.05 0.6 0.5];
        % ===== Frequenz der Simulation =====
        Y_x = fft(A(:,4));
        Y_y = fft(A(:,7));
        % Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
        L_fft = 2*floor(length(A)/2);
        P2_x = abs(Y_x/L_fft);
        P2_y = abs(Y_y/L_fft);

        P1_x = P2_x(1:L_fft/2+1);
        P1_y = P2_y(1:L_fft/2+1);

        P1_x(2:end-1) = 2*P1_x(2:end-1);
        P1_y(2:end-1) = 2*P1_y(2:end-1);
        % Define the frequency domain f and plot the single-sided amplitude spectrum P1. The amplitudes are not exactly at 0.7 and 1, as expected, because of the added noise. On average, longer signals produce better frequency approximations.

        f = (1/dt)*(0:(L_fft/2))/L_fft;
        axfft_SIM=subplot(1,4,4);
        plot(axfft_SIM,f,P1_x,f,P1_y,'--','LineWidth', 1)
        title('Single-Sided', 'Amplitude Spectrum','FontSize', 8)
        xlabel('f (Hz)')
        ylabel('|P1(f)|')
        legend(axfft_SIM,'$\theta$','$\phi$','Interpreter','latex','FontSize', 12)
        axis(axfft_SIM,[0 20 -inf inf])
        % ====== f Sim ende ======

        if Ermitteln == 1
        Schwingungsvergleich= figure('Name','Schwingungsvergleich','NumberTitle','off','Units','normalized');
        axSV=axes;
        VWSim=A(:,2)*360/(2*pi);
        plot(axSV,A(:,1),VWSim,S_W(:,1),S_W(:,S))
        title('Vergleich Winkelausschlag', string(S_Name),'Interpreter','none','FontSize', 8)
        legend(axSV,'Simulation','Messung')
        xlabel('Zeit [s]')
        ylabel('Winkel [ °]')
        grid(axSV,'minor')
        end
        % ================== Pendelpunkt ==========================
        Pendelpunkt=figure('Name','Pendelpunkt','NumberTitle','off','Units','normalized');
        ax21 = axes; %subplot(2,1,1);  % Linearbewegung
        plot(ax21...
            ,GOM_Ort(:,1),GOM_Ort(:,2),GOM_v(:,1),GOM_v(:,2),GOM_a(:,1),GOM_a(:,2)...
            ...,A(:,1),A(:,14),A(:,1),A(:,15),A(:,1),A(:,16)... X
            ...,A(:,1),A(:,17),A(:,1),A(:,18),A(:,1),A(:,19)... Y
            ...,A(:,1),A(:,20),A(:,1),A(:,21),A(:,1),A(:,22)... Z
            ...
            ...,A(:,1),A(:,14),'r',A(:,1),A(:,17),'g',A(:,1),A(:,20),'b'... Ort
            ...,A(:,1),A(:,15),'r',A(:,1),A(:,18),'g',A(:,1),A(:,21),'b'... Geschwindigkeit
            ...,A(:,1),A(:,16),'r',A(:,1),A(:,19),'g',A(:,1),A(:,22),'b'... Beschleunigung
            ...,GOM_IOrt(:,1),GOM_IOrt(:,2),GOM_IOrt(:,1),GOM_IOrt(:,3),GOM_IOrt(:,1),GOM_IOrt(:,4)...
            ...,GOM_Iv(:,1),GOM_Iv(:,2),GOM_Iv(:,1),GOM_Iv(:,3),GOM_Iv(:,1),GOM_Iv(:,4)...
            ...,GOM_Ia(:,1),GOM_Ia(:,2),GOM_Ia(:,1),GOM_Ia(:,3),GOM_Ia(:,1),GOM_Ia(:,4)...
            );
        legend(ax21...
            ,'x [m]','vx [m/s]','ax [m/s^2]'...
            ...,'y','vy','ay'...
            ,'z', 'vz','az'...
            ...
            ...,'x','y','z'...
            ...,'vx','vy','vz'...
            ...,'ax','ay','az'...
            );
        % legend(ax21,'ax [m/s^2]','vx [m/s]','x   [m]')
        title(ax21,'Bewegungsart: '+ string(Bewegungsart));
        maxax21=zeros(3,1);
        maxax21(1)=max(abs(min(GOM_Ort(:,2))),max(GOM_Ort(:,2)));
        maxax21(2)=max(abs(min(GOM_v(:,2))),max(GOM_v(:,2)));
        maxax21(3)=max(abs(min(GOM_a(:,2))),max(GOM_a(:,2)));
        maxax21=max(maxax21);
        axis(ax21,[0 tges -1.1*maxax21 1.1*maxax21]);
        ylabel(ax21,'Wert');
        xlabel(ax21,'t [s]');
        Pendelpunkt.OuterPosition(1:4) = [0 0.5 0.35 0.5];
        % ax22 = subplot(2,1,2);  % Zirkularbewegung
        % plot(ax22,A(:,1),A(:,27),A(:,1),A(:,28),A(:,1),A(:,29));
        % legend(ax22,'vB','azp','wB');
        % axis(ax22,[0 tges -inf inf]);
        % ylabel(ax22,'Wert');
        % xlabel(ax22,'t [s]');

        if Animation ==1 || Endergebnis ==1

            % Pendel im Raum
            PendelImRaum= figure('Name','Pendel im Raum','NumberTitle','off','Units','normalized');
            ax3 = axes;
            % Pendelnde Ebene
            Oberflaeche = figure('Name','Oberfläche','NumberTitle','off','Units','normalized');
            ax4 = axes;
        end
    end

    if Ermitteln == 1
        if flag_break_OS==1
            disp('Ansichtsfenster konnten nicht erstellt werden, da keine entsprechende Sensordatei ausgewählt wurde')
        else

            if GOM_Einbinden ==1
                % ================== GOM Daten anzeigen ========================
                GOM_Daten=figure('Name',string('Bewegungsdaten: '+string(GOM_Name)),'NumberTitle','off','Units','normalized');
                axGOM = axes;
                % plot(axGOM,data_GOM(:,1),data_GOM(:,3),data_GOM(:,1),data_GOM(:,4));%,data_GOM(:,1),data_GOM(:,4))
                hold on
                plot(axGOM,GOM_IOrt(:,1),GOM_IOrt(:,2),GOM_IOrt(:,1),GOM_IOrt(:,3),GOM_IOrt(:,1),GOM_IOrt(:,4));
                % plot(axGOM,GOM_a2(:,1),GOM_a2(:,2),GOM_a2(:,1),GOM_a2(:,3),GOM_a2(:,1),GOM_a2(:,4));
                % plot(axGOM,GOM_a(:,1),GOM_a(:,2),GOM_a(:,1),GOM_a(:,3),GOM_a(:,1),GOM_a(:,4));
                %plot(axGOM,GOM_Ia(:,1),GOM_Ia(:,2),GOM_Ia(:,1),GOM_Ia(:,3),GOM_Ia(:,1),GOM_Ia(:,4));
                legend(axGOM, ...
                    'X','Y','Z'...
                    ...,'Gläten1','Gläten2','Gläten3'...
                    );
                ylabel(axGOM,'Ort [m]');%'Beschleunigung [m/s^2]');
                xlabel(axGOM,'t [s]');
                GOM_Daten.OuterPosition(1:4) =   [0.5 0.1 0.5 0.8];
            end
            % ==================== Sensordaten anzeigen =======================
            if S_Ordner == "Ultraschallsensor"
                Sensor_Daten=figure('Name',string('Sensordaten: '+string(S_Name)),'NumberTitle','off','Units','normalized');
                axS = axes;
                % plot(axS,data_S(:,1),data_S(:,2),data_S(:,1),data_S(:,3),data_S(:,1),data_S(:,4))
                hold on
                % plot(axS,S_G(:,1),S_G(:,2),S_G(:,1),S_G(:,3),S_G(:,1),S_G(:,4))
                plot(axS,S_ts(:,1),S_ts(:,2),S_ts(:,1),S_ts(:,3),S_ts(:,1),S_ts(:,4))
                legend(axS, ...
                    ...'Z1','Z2','Z3',...
                    'Z1_G','Z2_G',...
                    'Z3_G'...
                    );
                grid(axS,'minor');
                title(axS,'Abstand zum Sensor');
                if tshift_SA ~= 0
                    title(axS,'Höhe des Messpunktes über Normalenfläche');
                end
                ylabel(axS,'Abstand [mm]');
                xlabel(axS,'t [s]');
                Sensor_Daten.OuterPosition(1:4) = [0 0.1 0.5 0.8];
            end

            % % Sensorwinkel Mathematisch parameter errechnet durch Daten und glättung
            % Zeitvektor = S_ts(:,1);
            % Sensorwinkel = S_ts(:,2);
            % ================= Sensorwinkel anzeigen ===========================

            % bereits in erster Ansicht

            %         Sensor_Winkel=figure('Name',string('Sensor_Winkel: '+string(S_Name)),'NumberTitle','off','Units','normalized');
            %         axSWtheta = subplot(2,1,1);
            %         plot(axSWtheta,S_WINT(:,1),S_WINT(:,2),'b',S_WINT(:,1),S_WINT(:,2),'r.'...
            %             ,'MarkerSize',3.5,'LineWidth',0.3);
            %         lim_theta = max(abs(min(S_WINT(:,2))),max(S_WINT(:,2)));
            %         axis(axSWtheta,[0 inf -lim_theta lim_theta])
            %         title(axSWtheta,'Sensorwinkel');
            %         ylabel(axSWtheta,'Winkel [ °]');
            %         xlabel(axSWtheta,'t [s]');
            %         legend(axSWtheta,'Theta','Datenpunkt');
            %         hold on
            %         axSWphi = subplot(2,1,2);
            %         plot(axSWphi,S_WINT(:,1),S_WINT(:,3),'r',S_WINT(:,1),S_WINT(:,3),'b.'...
            %             ,'MarkerSize',3.5,'LineWidth',0.3);
            %         lim_phi = max(abs(min(S_WINT(:,3))),max(S_WINT(:,3)));
            %         axis(axSWphi,[0 inf -lim_phi lim_phi])
            %         ylabel(axSWphi,'Winkel [ °]');
            %         xlabel(axSWphi,'t [s]');
            %         legend(axSWphi,'Phi','Datenpunkt');
            %         if tshift_Bew ~=0
            %             patch(axSWtheta,[tshift_Bew,tshift_Bew+Bewegungszeit...
            %                 ,tshift_Bew+Bewegungszeit,tshift_Bew]...
            %                 ,[-lim_theta -lim_theta lim_theta lim_theta],[0.9290 0.6940 0.1250] ...
            %                 ,FaceAlpha = 0.3, EdgeAlpha = 0)
            %             patch(axSWphi,[tshift_Bew,tshift_Bew+Bewegungszeit...
            %                 ,tshift_Bew+Bewegungszeit,tshift_Bew]...
            %                 ,[-lim_theta -lim_theta lim_theta lim_theta],[0.9290 0.6940 0.1250] ...
            %                 ,FaceAlpha = 0.3, EdgeAlpha = 0)
            %             legend(axSWtheta,'Theta','Datenpunkt','Bewegungszeit');
            %             legend(axSWphi,'Phi','Datenpunkt','Bewegungszeit');
            %         end
        end
    end
    % Faktor für meshgrid an L anpassen und für Kameraview/target
    [X,Y] = meshgrid(-R:2*R:R,-R:2*R:R);
break
end

% In Prozent von Bildschirmgröße         x,  y, w, h

%=======================================
% ======Modellierung Pendelverlauf======
%=======================================
if Simulieren==1 && Animation==1
    PendelImRaum.OuterPosition(1:4) = [0.6 0 0.4 1];
    Oberflaeche.OuterPosition(1:4) = [0.35 0.5 0.25 0.5];
    %     if A(1,2)>A(1,5); n=sind(A(1,2));% hier nach größter
    %     else n=sind(A(1,5)); end         % Anfangsauslenkung in einer Richtung
    j = 1;
    lj = j;  %  150s bei dt 0.05 ohne dauerte 8:00+ min, mit 4:00 min
    timer = true;
    %     F(tges/dt) = struct('cdata',[]); % Bildarray für Video
    while j <= tges/dt
        if j==1, tStart = tic;end
        tic;
        figure(PendelImRaum);
        delete(findall(gcf,'type','annotation'));
        delete(findall(gcf,'type','quiver'));
        %=========
        daspect(ax3,[1 1 1]);
        annotation(PendelImRaum,'textbox',[0.1, 0.85, 0.1, 0.1],'FitBoxToText','on','String',{"Tges = "+string(round(tges,2))+"s",'t: '+string(round(A(j,1),2))+"s",string(round(T(end,1),2))+' / '+FT})
        annotation(PendelImRaum,'textbox',[0.3, 0.85, 0.1, 0.1],'FitBoxToText','on','String',{'L = '+string(L)+' [m]','M= '+string(M)+' [kg]','d = '+string(d)+' [N/(m/s)]'})
        title(ax3,'Amplitudenverlauf')
        xlabel(ax3,'x[m]'); ylabel(ax3,'y[m]'); zlabel(ax3,'z[m]');
        % Kamera
        if CamMoving == 1
            campos([A(j,14) A(j,17) A(j,20)]+Blickposition)
            camtarget([A(j,14) A(j,17) A(j,20)])
            axis(ax3,[A(j,14)-n A(j,14)+n A(j,17)-n A(j,17)+n A(j,20)-L-n A(j,20)]);
        else
            campos(+Blickposition)
            camtarget([0 0 0])
            axis(ax3,[-n n -n n -L 0]);
        end
        % ================
        %Verlauf Boden
        plot3(ax3,A(lj:j,14)+A(lj:j,11),A(lj:j,17)+A(lj:j,12),B(lj:j,1),'-black');
        if j==1, grid(ax3,'minor'),end
        hold on
        %==============
        % Verlauf im Raum
        plot3(ax3,A(lj:j,14)+A(lj:j,11),A(lj:j,17)+A(lj:j,12),A(lj:j,20)+A(lj:j,13)... %... Ortspunkte der Pendelmasse
            ,'-blue')%,'MarkerSize',15,'MarkerEdgeColor','red','MarkerFaceColor','red',"MarkerIndices",j);%,... Darstellung der aktuellen Pendelmasse
        %Probleme mit gestapelter Darstellung von lj-j, weil Marker
        %bestehen bleibt und sich nicht löschen lies
        % Modellierung Pendelstab
        quiver3(ax3,A(j,14),A(j,17),A(j,20),A(j,11),A(j,12),A(j,13)...%... Vektor für den Pendelstab   %,'black'...
            ,'vblack','MarkerSize',7,'MarkerEdgeColor','black','MarkerFaceColor','black'...%Stellt Aufhängepunkt des Pendels an dem Vektor für den Pendelstab dar
            ,'AutoScale','off','LineWidth', 1.5)
        quiver3(ax3,A(j,14),A(j,17),A(j,20),0,0,-L,'--.black','AutoScale','off')  %Lot vom Pendelpunkt
        quiver3(ax3,A(j,11)+A(j,14),A(j,12)+A(j,17),-L,-A(j,11),-A(j,12),0,':red','LineWidth',1,'AutoScale',0,'ShowArrowHead','on')%'off') % Bodenspur der Startrichtung
        legend('Projektion Boden',...
            'Spur',...
            'Pendelstab',...
            'Lot',...
            'Auslenkrichtung');
        daspect(ax3,[1 1 1]);
        % %         F(j) = getframe(f2);
        % Flüssigkeit in Zylinder
        if Zylinder == 1
            % Kreiskoordinaten für Oberfläche
            X1=(-R:R/100:R); X2=(-R:R/100:R);
            Y1=sqrt(R^2-X1.^2); Y2=-sqrt(R^2-X2.^2);
            Z1 = -X1.*(A(j,23)/A(j,25))-Y1.*(A(j,24)/A(j,25)); Z2 = -X2.*(A(j,23)/A(j,25))-Y2.*(A(j,24)/A(j,25));
            % Boden
            ZB = -H+X1.*0;
            % Wände
            XW1=[X1,flip(X1)];
            YW1=[Y1,flip(Y1)];
            ZW1=[ZB,flip(Z1)];
            %         fill3(ax4,XW1,YW1,ZW1,'cyan');
            XW2=[X2,flip(X2)];
            YW2=[Y2,flip(Y2)];
            ZW2=[ZB,flip(Z2)];
            %         fill3(ax4,XW2,YW2,ZW2,'cyan');
            fill3(ax4,X1,Y1,Z1,'cyan',X2,Y2,Z2,'cyan',X1,Y1,ZB,'cyan',X2,Y2,ZB,'cyan',XW1,YW1,ZW1,'cyan',XW2,YW2,ZW2,'cyan')
            axis(ax4,[-R-R/10 R+R/10 -R-R/10 R+R/10 -H H])
            daspect(ax4,[1 1 1])
            %         surf(ax4,X,Y,Z,'FaceColor','interp')
            %         axis(ax4,[-n*L n*L -n*L n*L -n*L n*L])
            %daspect(ax4,[1 1 1]);
            %view(ax4,0,0)
        end
        % Oberfläche rechteckig, X/Y durch Meshgrid
        if Zylinder == 0
            Z = -X.*(A(j,23)/A(j,25))-Y.*(A(j,24)/A(j,25));
            surf(ax4,X,Y,Z,'FaceColor','interp')
            axis(ax4,[-R-R/10 R+R/10 -R-R/10 R+R/10 -2*H 2*H])
            daspect(ax4,[1 1 1])
            colormap(ax4,'winter');
            caxis(ax4,[-10/360*2*pi*R 10/360*2*pi*R]) % Farbwertebereich für colorbar festlegen (ändert sich dann nicht dyn nach größtem z-Wert
        end
        lj = j;
        if j==floor(tges/FT)*FT/dt+1; j=tges/dt+1; else j=j+FT/dt; end
        %nicht jeden wert ausgeben, da sonst zu langsam, framerate ca. 0.2sec
        % if-Bedingung, damit auch letzter Arraywert abgefangen wird, weil das
        % sonst mit der teilung von fr/dt nicht aufgeht wenn bei j=1
        % angefangen wird
        t_i=toc;
        T=[T;t_i]; %
        if t_i<FT, pause(FT-t_i),end   %verstrichene berechnungszeit von Zeit der framerate abziehen
    end
    tEnd = toc(tStart);% Erste Zeitmessung abziehen, da >Durchschnitl. lang
    tEnd = tEnd-T(2);
    if ZeitAN ==1
        SimZeit= figure('Name','Berechnungszeit pro Simulationsschritt','NumberTitle','off','Units','normalized');
        axSimZeit =axes;
        t=[A(1:(FT/dt):end,1);tges]; %letzter Datenpunkt ist bei tges, was p*fr/dt-1 ist
        Tgrenz = FT*ones(length(t),1); % Anzeige fr-Zeit Grenze
        plot(axSimZeit,t(3:length(T),1),T(3:length(T),1),t(3:end,1),Tgrenz(3:end,1),'r');  % T mit end-1, manchmal
        axis(axSimZeit, [0 tges 0 0.3]);
        ylabel(axSimZeit,'Dauer Iterationsschritt [s]');
        xlabel(axSimZeit,'t [s]');
        annotation(SimZeit,'textbox',[0.15, 0.8, 0.1, 0.1],'String',"tges = "+string(round(tges,2))+"s")
        annotation(SimZeit,'textbox',[0.15, 0.735, 0.1, 0.1],'String',"tEnd = "+string(round(tEnd,2))+"s")
        annotation(SimZeit,'textbox',[0.15, 0.67, 0.1, 0.1],'String',"Abweichung = "+string(round((tEnd/(tges-FT)-1)*100,2))+"%")
    end
    % movie(fig,F,1/fr);
end
% figure(PendelImRaum);

% Nur endergebnis
while CodeFalten ==1
    if Endergebnis==1 && Animation ==0
        if GOM_Einbinden == 0
            j=tges/dt+1;
        else
            j=length(A);
        end
        figure(PendelImRaum);
        plot3(ax3,A(1:j,14)+A(1:j,11),A(1:j,17)+A(1:j,12),B(1:j,1),'-black');
        %     axis(ax3,[A(j,14)-n*L A(j,14)+n*L A(j,17)-n*L A(j,17)+n*L A(j,20)-L A(j,20)]);
        daspect(ax3,[1 1 1]);
        hold on
        plot3(ax3,A(1:j,14)+A(1:j,11),A(1:j,17)+A(1:j,12),A(1:j,20)+A(1:j,13)... %... Ortspunkte der Pendelmasse
            ,'-oblue','MarkerSize',15,'MarkerEdgeColor','red','MarkerFaceColor','red',"MarkerIndices",j);
        quiver3(A(j,14),A(j,17),A(j,20),A(j,11),A(j,12),A(j,3)...%... Vektor für den Pendelstab   %,'black'...
            ,'vblack','MarkerSize',10,'MarkerEdgeColor','black','MarkerFaceColor','black'...%Stellt Aufhängepunkt des Pendels an dem Vektor für den Pendelstab dar
            ,'AutoScale','off','LineWidth', 1.5);
        quiver3(ax3,A(j,14),A(j,17),A(j,20),0,0,-L,'--.green','AutoScale','off');  %Lot vom Pendelpunkt
        quiver3(ax3,A(1,11),A(1,12),-L,-2*A(1,11),-2*A(1,12),0,'--.black','AutoScale','off'); %Anfangsorientierung durch Lotpunkt
        title(ax3,'Amplitudenverlauf');
        grid(ax3,'minor');
        xlabel(ax3,'x');
        ylabel(ax3,'y');
        zlabel(ax3,'z');
        legend('Projektion Boden','Spur','Pendelstab','Lot','Auslenkrichtung');
    end
break
end

% Pendel an bestimmter Position
while CodeFalten ==1
    if Pendelposition==1
        theta = A(1,2);
        phi = A(1,5);

        % Rotationsmatrizen
        L_v=[0; 0; -L];
        R_x=[1 0 0; 0 cosd(phi), sind(phi); 0, -sind(phi), cosd(phi)];
        R_y=[cosd(theta), 0, sind(theta); 0, 1, 0; -sind(theta), 0 cosd(theta)];
        Pos=R_x*R_y*L_v;
        % Funktionieren nicht, dalängung der bereits verschobenen Achse durch
        % drehung der anderen Achse falsch berücksichtigt wird

        a = cosd(phi)*sind(theta);
        b = sind(phi)*cosd(theta);
        c = -cosd(phi)*cosd(theta);
        Lnorm = sqrt(a^2+b^2+c^2);
        % x_P
        x_p = L/Lnorm * a;
        % y_P
        y_p = L/Lnorm * b;
        % z_P
        z = L/Lnorm * c;
        figure(PendelImRaum);
        quiver3(ax3,0,0,0,x,y,z...%... Vektor für den Pendelstab   %,'black'...
            ,'vblack','MarkerSize',10,'MarkerEdgeColor','black','MarkerFaceColor','black'...%Stellt Aufhängepunkt des Pendels an dem Vektor für den Pendelstab dar
            ,'AutoScale','off','LineWidth', 1.5);
        %     view(90,0);
        axis(ax3,[-L L -L L -L L])
        daspect(ax3,[1 1 1]);
    end
    break
end

function [Ziel_Ort] = Koordinatenkorrektur(Ort,Ordner) % X-Y-Z-Koordinatenrichtungen anpassen
if 1 == strcmp(Ordner,'GOM')
    Ort(:,4) = -Ort(:,4);
    Ziel_Ort(:,[1,2,4,3]) = Ort(:,1:4);
    % GOM_X -- X
    % GOM_Y -- Z
    % GOM_Z -- -Y
else
    Ziel_Ort=Ort;
end
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

function [Erg] = Integrieren(data)
Erg = zeros(length(data),width(data));
Erg(:,1) = data(:,1);
dt = data(2,1)-data(1,1);
j=1;        % v=a*t+v_0 oder s=v*t+s_0
while j<length(Erg)
    Erg(j+1,2) = data(j+1,2)*dt +Erg(j,2);
    Erg(j+1,3) = data(j+1,3)*dt +Erg(j,3);
    Erg(j+1,4) = data(j+1,4)*dt +Erg(j,4);
    j=j+1;
end
end

function [u,v,w] = KoordTrans(r,aB,as,PM)
u = r*cos(aB);  % X Koordinate für Wegfortschritt, bei X+ anfangend nach Y+drehend
v = r*sin(aB);  % Y- Koordinate
w = 0;  % evtl Z-Komponente
K = ([u;v;w]);
Krot = (([cos(as), -sin(as),0; sin(as), cos(as),0; 0,0,1 ])*K); %Drehmatrix
Ktransl = Krot+PM;
u=Ktransl(1);
v=Ktransl(2);
w=Ktransl(3);
end

function [Erg] = SensorWinkel(data,ds)
Erg = zeros(length(data),width(data));
Erg(:,1:4) = data(:,1:4);
j=1;
while j<=length(data)
    % Oberfläche Sensoren
    Z1 = data(j,2);
    Z2 = data(j,3);
    Z3 = data(j,4);

    % Nur Winkel in X-Richtung
    Z13 = Z1-Z3;
    Erg(j,8) = Z13;
    X13 = 2*(sind(60)*ds);
    XWinkel = atand(Z13/X13);
    % Winkel in Y-Richtung
    Zmitte2 = (Z1+Z3)/2;
    Z2m = Z2-Zmitte2;
    X2m = cosd(60)*ds+ds;
    YWinkel = atand(Z2m/X2m);
    Erg(j,5) = XWinkel;
    Erg(j,6) = YWinkel;
    Erg(j,7) = 0;    % Sloshing Angle

    % Aus Ebenengleichung ( Fehler)
    %  Aordnung in Richtungsvektoren vom Mittelpunkt aus
    O1 = ([-sind(60)*ds;  cosd(60)*ds; Z1]);
    O2 = ([0;  ds;  Z2]);
    O3 = ([sind(60)*ds;   cosd(60)*ds;  Z3]);
    V12 = O2-O1;
    V13 = O3-O1;
    nSensor = cross(V12,V13);
    % Ebenengleichung mit Punkt (0/0/0) als Element
    % nSensor * X = 0,
    % Vektor in Oberflächenebene entlang X-Z Ebene
    t0 = ([1; 0; 0]);
    tx = ([1; 0; -nSensor(1)/nSensor(3)]);
    % Vektor in Oberflächenebene entlang X-Z Ebene
    p0 = ([0; 1; 0]);
    py = ([0; 1; -nSensor(2)/nSensor(3)]);
    thetaS = acos(norm(dot(t0,tx))/(norm(t0)*norm(tx)));
    phiS = acos(norm(dot(p0,py))/(norm(p0)*norm(py)));
    if -nSensor(1)/nSensor(3)>0, thetaS=-thetaS; end
    if -nSensor(2)/nSensor(3)>0, phiS=-phiS; end
    %Erg(j,8) = thetaS;
    %Erg(j,9) = phiS;


    j=j+1;
end
end

function [Erg] = Saeubern(data)
Fill = zeros(length(data),width(data));
j=1;
k=1;
while j<=length(data)
    if data(j,2)~=0  && data(j,3)~=0 %&& ~isnan(data(j,2)) %&& data(j,3)~=0
        Fill(k,:) = data(j,:);
        k=k+1;
    end
    j=j+1;
end
Erg = Fill(1:k-1,:);
end

function [Erg] =FindIdx(data,Zeit)
for idx=1:length(data)
    if data(idx,1) >= Zeit
        Erg = idx;
        break
    end
end
end

function [Erg] = Glaetten(data,mw)
Erg = zeros(length(data),width(data));
Erg(:,1) = data(:,1);
mw= floor(mw/2);
j=1;
while j<=length(data)
    sum1 = 0;
    sum2 = 0;
    sum3 = 0;
    if j<=mw
        for k= 0:(j-1+mw)
            sum1 = sum1 + data((1+k),2);
            sum2 = sum2 + data((1+k),3);
            sum3 = sum3 + data((1+k),4);
        end
    elseif j> (length(data)-mw)
        for k= 0:(length(data)-j+mw)
            sum1 = sum1 + data((j-mw+k),2);
            sum2 = sum2 + data((j-mw+k),3);
            sum3 = sum3 + data((j-mw+k),4);
        end
    else
        for k= 0:(2*mw)
            sum1 = sum1 + data((j-mw+k),2);
            sum2 = sum2 + data((j-mw+k),3);
            sum3 = sum3 + data((j-mw+k),4);
        end
    end
    Erg(j,2) = sum1/(k+1);
    Erg(j,3) = sum2/(k+1);
    Erg(j,4) = sum3/(k+1);
    j=j+1;
end
end

function [Erg] = Kalibrieren_S(data_S,idx)
j=1;
sum1 = 0;
sum2 = 0;
sum3 = 0;
while j<=idx
    sum1 = sum1 + data_S(j,2);
    sum2 = sum2 + data_S(j,3);
    sum3 = sum3 + data_S(j,4);
    j=j+1;
end
Erg1 = sum1/j;
Erg2 = sum2/j;
Erg3 = sum3/j;
% Sensor bias verrechnen
% Ergibt Abweichung um Zielentfernung herum
if idx ~= 0
    Erg = [data_S(:,1),...
        ones(length(data_S),1)*Erg1 - data_S(:,2),...
        ones(length(data_S),1)*Erg2 - data_S(:,3),...
        ones(length(data_S),1)*Erg3 - data_S(:,4)];
end
end

function [Erg] = Kalibrieren_S_T(data_S,idx)
j=1;
sum1 = 0;
sum2 = 0;
while j<=idx
    sum1 = sum1 + data_S(j,2);
    sum2 = sum2 + data_S(j,3);
    j=j+1;
end
Erg1 = sum1/j;
Erg2 = sum2/j;
% Sensor bias verrechnen
% Ergibt Abweichung um Zielentfernung herum
if idx ~= 0
    Erg = [data_S(:,1),...
        data_S(:,2)-ones(length(data_S),1)*Erg1,...
        data_S(:,3)-ones(length(data_S),1)*Erg2,zeros(length(data_S),1)];
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

function [Erg] = W_Name(Spaltenindex)
switch Spaltenindex
    case 1
        Erg="Zeit";
    case 2
        Erg ="Theta";
    case 3
        Erg ="Phi";
    otherwise
        Erg="Spaltenindex überprüfen";
end
end

function [GOM, Bewegungszeit] = BewegungsartZuordnen(GOM, GOM_Name, Bewegungsart)
    switch  GOM_Name
        case 'Pendel_Versuche Ort und Beschleunigung.csv'
            % Fehlerhafte Daten im Datenbereich für Bewgung mit negativer Z-Richtung.
            % Beschleunigungsverlauf ist aber außreichend genaue für Bewegung in +/-
            % Z-richtung, daher Spiegelung der positiven Bewegungsrichtung möglich
            % Aufteilung Bewegungsfall
            GOM = GOM(1:1895,:); %1900
            GOM = [GOM; flipud(GOM(1750:1895,:))];
            switch Bewegungsart  %koord, linear, orthogonal, z-positiv, z-negativ
                case 'koord'
                    GOM_s = 100;
                    GOM_e = 400;
                    Bewegungszeit = 0;
                case 'linear'
                    GOM_s = 601;
                    GOM_e = 780;
                    Bewegungszeit = 2.3;
                case 'orthogonal'
                    GOM_s = 1070;
                    GOM_e = 1225;
                    Bewegungszeit = 2.04;
                case 'z-positiv'
                    GOM_s = 1745;
                    GOM_e = 1895;
                    Bewegungszeit = 2.8;
                case 'z-negativ'
                    GOM_s = 1896; % 2069
                    GOM_e = length(GOM); % 2219
                    Bewegungszeit = 2.8;
                otherwise
                    GOM_s = 1;
                    GOM_e = length(GOM);
                    Bewegungszeit = 0; % 1.2 Emily_soll, 1.5 gemessen
                    disp('Kein GOM Bereich ausgewählt für "'+string(GOM_Name)+'"')
            end
            GOM = GOM(GOM_s:GOM_e,:);

        case 'Emily_2ms_075_08a.csv'
            switch Bewegungsart  % linear, neben, z-positiv, z-negativ
    
                case 'linear'
                    GOM_s =0.576;%49;%0.576
                    GOM_e = 3.588;%300;%3.588
                    Bewegungszeit = GOM_e-GOM_s;
                case 'neben'
                    GOM_s = 11.796;%984;%11.796
                    GOM_e = 14.796;%1234;%14.796
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-positiv'
                    GOM_s = 36.315;%3028;%36.324
                    GOM_e = 40.404;%3368;%40.404
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-negativ'
                    GOM_s = 49.452;%4122;%49.452
                    GOM_e = 52.5;%4376;%52.5
                    Bewegungszeit = GOM_e-GOM_s;
                otherwise
                    GOM_s = 0;
                    GOM_e = 0;
                    Bewegungszeit = 0; % 1.2 Emily_soll, 1.5 gemessen
                    disp('Kein GOM Bereich ausgewählt für "'+string(GOM_Name)+'"')
            end

        case 'Emily_2ms_09_08a.csv'
            switch Bewegungsart  % linear, neben, z-positiv, z-negativ
                % Zeitpunkte angeben
                case 'linear'
                    GOM_s =0.57;
                    GOM_e = 3.12;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'neben'
                    GOM_s = 10.84;
                    GOM_e = 13.39;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-positiv'
                    GOM_s = 31.61;
                    GOM_e = 34.37;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-negativ'
                    GOM_s = 42.65;
                    GOM_e = 45.29;
                    Bewegungszeit = GOM_e-GOM_s;
                otherwise
                    GOM_s = 0;
                    GOM_e = 0;
                    Bewegungszeit = 0; % 1.2 Emily_soll, 1.5 gemessen
                    disp('Kein GOM Bereich ausgewählt für "'+string(GOM_Name)+'"')
            end

            case 'Emily_2ms_1_1a.csv'
            switch Bewegungsart  % linear, neben, z-positiv, z-negativ
                % Zeitpunkte angeben
                case 'linear'
                    GOM_s = 0.57;
                    GOM_e = 2.66;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'neben'
                    GOM_s = 9.94;
                    GOM_e = 12.02;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-positiv'
                    GOM_s = 32.12;
                    GOM_e = 34.24;
                    Bewegungszeit = GOM_e-GOM_s;
                case 'z-negativ'
                    GOM_s = 42.13;
                    GOM_e = 44.26;
                    Bewegungszeit = GOM_e-GOM_s;
                otherwise
                    GOM_s = 0;
                    GOM_e = 0;
                    Bewegungszeit = 0; % 1.2 Emily_soll, 1.5 gemessen
                    disp('Kein GOM Bereich ausgewählt für "'+string(GOM_Name)+'"')
            end
                
        otherwise
        GOM_s = 0;
        GOM_e = 0;
        Bewegungszeit = 0;
        warning('Für "'+string(GOM_Name)+'" ist kein Profil angelegt')
    end
     % Indizes der Zeitpunkte ermitteln, GOM auf diesen Bereich stutzen
     try
        if GOM_s ~=0 && GOM_e ~= 0
        [GOM_st] =FindIdx(GOM(:,1),GOM_s);
        [GOM_et] =FindIdx(GOM(:,1),GOM_e);
        GOM = GOM(GOM_st:GOM_et,:);
        GOM(:,1)=GOM(:,1)-ones(length(GOM),1)*GOM(1);
        end
     catch
        
     end
end