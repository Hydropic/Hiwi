function  VisualizeAll(robot,achsstellung,beschleunigung,preserve_plot)

    %berechnung des TCP punktes aus den Achsstellungen
    [tcppunkte(1,:)] = vorwaertskinematik(achsstellung(1,:));

    %plotten des beschleunigungsvektors
    quiver3(tcppunkte(1),tcppunkte(2),tcppunkte(3),beschleunigung(1)*5,beschleunigung(2)*5,beschleunigung(3)*5,'color','g','LineWidth',2)
    
    %plotten der Vorzugsrichtung und des Korridores
    laufbahn(robot,achsstellung,1,preserve_plot)

    
end

