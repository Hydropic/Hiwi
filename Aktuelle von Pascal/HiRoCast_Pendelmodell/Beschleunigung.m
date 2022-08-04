%Funktion, um die Beschleunigung des TCPs zu berechnen 
function [beschleunigungSkalar,beschleunigungVektor,beschleunigungVektorZumZeichnen] = Beschleunigung(vorgaenger,punkt,nachfolger,zeit,MatrixKelleUngekippt)
    Vektor1 = punkt-vorgaenger;
    Strecke1 = sqrt(Vektor1(1)^2+Vektor1(2)^2+Vektor1(3)^2);
    geschwindigkeitSkalar = Strecke1/zeit;
    geschwindigkeitVektor = Vektor1/zeit;

    %RichtungGlobal beschreibt die Beschleunigungsrichtung als Vektor im
    %globalen Koordinatensystem zwischen den Punkten der Bahn, zum
    %Vergleich mit den Werten der Fluidtabelle benötigen wir aber die
    %Beschleunigungsrichtung als Vektor in Relation zu der Rotation der
    %Kelle, d.h. wird hierfür die vorwaertskinematik genutzt
    Vektor2 = nachfolger-punkt;
    Strecke2 = sqrt(Vektor2(1)^2+Vektor2(2)^2+Vektor2(3)^2);
    beschleunigungSkalar = (Strecke2-geschwindigkeitSkalar*zeit)/(zeit^2*0.5);
    beschleunigungVektor = (Vektor2-geschwindigkeitVektor*zeit)/(zeit^2*0.5);
    
    beschleunigungVektorZumZeichnen = beschleunigungVektor;

    %Vektor in Koordinatensystem der Fluidgruppe umtransformieren
    NullVektor = [0 0 0];
    FluidGlobMatrix = [MatrixKelleUngekippt NullVektor';
                       NullVektor               1     ];
    beschleunigungVektor = [beschleunigungVektor; 
                                    0            ];
    beschleunigungVektor = FluidGlobMatrix*beschleunigungVektor;

    %AltLast die vielleicht unnötig:
    %[a,b,RichtungInTCPKoordinaten] = vorwaertskinematik(achsstellung,beschleunigungVektor);
end 