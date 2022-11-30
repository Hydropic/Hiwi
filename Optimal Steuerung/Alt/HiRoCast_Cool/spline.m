%----------------------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------------------
%----------------------------------------------------------------------------------------------------------
%Funktion für den Spline für eine Achse 
%input: Thetawerte für die entsprechende Achse, Abstände zwischen den Thetawerten,
%boolean Angabe, ob die Ausgabe geplottet werden soll 
%Schrittanzahl kann geändert werden (nicht zuwenige, da sonst kein Spline
%sinnvoller Spline entsteht)
%output: Spline, Geschwindigkeit, Beschleunigung, Ruck, Zeiten (zu jedem
%Arrayeintrag den t-Wert), passendeStelle (Arraypositionen der
%Input-Punkte)
function [spline,geschwindigkeit,beschleunigung,ruck,time,passendeStelle] = spline(punkte,h,plotten,varargin)
    %Anzahl der Segmente (n) berechnen 
    n = length(h);
    
    %Faktor für die Anzahl der Schritte zwischen zwei ganzen Zahlen 
    if nargin  == 4
        Schritte = varargin{1};
    elseif nargin == 3  
        Schritte = 1000;
    end
    Schrittgroesse = 1/Schritte;
    
    %Zeitpunkte (t) berechnen 
    t = zeros(1,n+1);
    for i= 2:n+1
        for j = 1:i-1
            t(i) = h(j)+t(i);
        end 
    end 

    %Werte für TDMA nach Wikipedia 
    %Größe der Matrix
    laenge = n-1;
    %obere Diagonale füllen
    up = zeros(1,laenge-1);
    for i = 2:laenge
        up(i-1) = h(i);
    end 
    %untere Diagonale füllen 
    down = [0,up];
    %Diagonale füllen 
    dia = zeros(1,laenge);
    for i = 1:laenge
        dia(i) = 2*(h(i)+h(i+1));
    end
    %rechte Seite berechnen 
    right = zeros(1,laenge);
    for i = 1:laenge
        right(i) = 6*(((punkte(i+2)-punkte(i+1))/h(i+1))-((punkte(i+1)-punkte(i))/h(i)));
    end
    
    %TDMA
    for k = 2:laenge
        factor = down(k)/dia(k-1);
        dia(k) = dia(k)-factor*up(k-1);
        right(k) = right(k)-factor*right(k-1);
    end
    
    ableitungen2 = zeros(1,laenge);
    ableitungen2(laenge) = right(laenge)/dia(laenge);
    
    for k = laenge-1:-1:1
        ableitungen2(k) = (right(k)-up(k)*ableitungen2(k+1))/dia(k);
    end
    
    %Varibalen für qerg bestimmen
    d = punkte(1:n);
    b = [0,ableitungen2/2,0];
    a = zeros(1,n);
    for i = 1:n
        a(i) = (2*b(i+1)-2*b(i))/(6*h(i));
    end
    c = zeros(1,n);
    for i = 1:n
        c(i) = ((punkte(i+1)-punkte(i))/h(i))-a(i)*(h(i))^2-b(i)*h(i);
    end
   
    %spline = zeros(1,t(n+1)*Schritte);
    %geschwindigkeit = zeros(1,t(n+1)*Schritte);
    %beschleunigung = zeros(1,t(n+1)*Schritte);
    %ruck = zeros(1,t(n+1)*Schritte);
    
    groesse = 0; 
    %Größe der Arrays berechnen
    for i = 1:n
        for j = t(i):Schrittgroesse:(t(i+1)-Schrittgroesse)
            groesse = groesse+1;
        end
    end
    
    %Arrays für die Funktionswerte, 1. bis 3. Ableitung erstellen 
    count = 0;
    spline = zeros(1,groesse);
    geschwindigkeit = zeros(1,groesse);
    beschleunigung = zeros(1,groesse);
    ruck = zeros(1,groesse);
    %Position im Array, an der die ursprünglichen Punkte zu finden sind 
    passendeStelle = zeros(1,n+1);
    time = zeros(1,groesse);

    %Funktionswerte, 1. Ableitung, 2. Ableitung und 3. Ableitung berechnen 
    for i = 1:n
        nulltest = 0; 
        for j = t(i):Schrittgroesse:(t(i+1)-Schrittgroesse)
            nulltest = 1; 
            count = count+1;
            spline(count) = a(i)*(j-t(i))^3+b(i)*(j-t(i))^2+c(i)*(j-t(i))+d(i);
            geschwindigkeit(count) = 3*a(i)*(j-t(i))^2+2*b(i)*(j-t(i))+c(i);
            beschleunigung(count) = 6*a(i)*(j-t(i))+2*b(i);
            ruck(count) = 6*a(i);
            time(count) = j; 
            if j == t(i)
                passendeStelle(i) = count;
            end 
        end
        %Wert einfügen, falls die Schleife zu klein ist und nichts einfügt 
        if nulltest == 0
            count = count+1;
            spline(count) = d(i);
            geschwindigkeit(count) = c(i);
            beschleunigung(count) = 2*b(i);
            ruck(count) = 6*a(i);
            time(count) = t(i); 
            passendeStelle(i) = count; 
        end 
    end
    passendeStelle(n+1) = length(spline);
    
    if plotten == true
        %Plotten der gegebenen Punkte und des Splines 
        figure
        subplot(4,1,1)
        hold on
        plot(t,punkte, "linestyle", "none","marker","o");
        plot(time,spline, "Color","green");
        title("Spline");
        xlabel("Time");
        ylabel("Angle");
        hold off
        
        %Plotten der 1. Ableitung/Geschwindigkeit
        subplot(4,1,2)
        plot(time,geschwindigkeit, "Color","magenta");
        title("Velocity");
        xlabel("Time");
        ylabel("Velocity");
        
        %Plotten der 2. Ableitung/Beschleunigung 
        subplot(4,1,3)
        hold on 
        plot(time,beschleunigung, "Color","yellow");
        title("Acceleration");
        xlabel("Time");
        ylabel("Acceleration");
        %Plottet nur die Ableitungen der Punkte 
        %plot(ti,[0,ableitungen2,0],"marker","o");
        hold off 
        
        %Plotten der 3. Ableitung/Ruck
        subplot(4,1,4) 
        plot(time,ruck, "Color","red");
        title("Jerk");
        xlabel("Time");
        ylabel("Jerk");
    end
end 
