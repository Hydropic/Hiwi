function show_spline(solution, window_title)
    timeintervals = solution(1:size(solution,1)-1,1);
    base_points = solution(:,2:size(solution,2));
    dlmwrite("basepoints.txt",base_points); %#ok<*DLMWT> 
    dlmwrite("timeintervals.txt",timeintervals);

    figure('name', window_title);
    for i=1:size(base_points,2)
        fprintf('SPLINE %d',i);
        [t,td,tdd,tddd,time,place] = spline(base_points(:,i)',timeintervals,false);
        
        subplot(4,size(base_points,2),1+(i-1))
        hold on
        plot(compute_time(timeintervals),base_points(:,i), "linestyle", "none","marker","o");
        plot(time,t, "Color","green");
        title("Spline ",i);
        xlabel("Time");
        ylabel("Angle");
        ax = gca;
        ax.YAxis.Exponent = 0;
        hold off
     
        %Plotten der 1. Ableitung/Geschwindigkeit
        subplot(4,size(base_points,2),7+(i-1))
        plot(time,td, "Color","magenta");
        title("Velocity");
        xlabel("Time");
        ylabel("Velocity");
        ax = gca;
        ax.YAxis.Exponent = 0;
        
        %Plotten der 2. Ableitung/Beschleunigung 
        subplot(4,size(base_points,2),13+(i-1))
        hold on 
        plot(time,tdd, "Color","yellow");
        title("Acceleration");
        xlabel("Time");
        ylabel("Acceleration");
        %Plottet nur die Ableitungen der Punkte 
        %plot(ti,[0,ableitungen2,0],"marker","o");
        ax = gca;
        ax.YAxis.Exponent = 0;
        hold off 
        
        %Plotten der 3. Ableitung/Ruck
        subplot(4,size(base_points,2),19+(i-1)) 
        plot(time,tddd, "Color","red");
        title("Jerk");
        xlabel("Time");
        ylabel("Jerk");
        ax = gca;
        ax.YAxis.Exponent = 0;
    
    end
    fprintf('Zeit: %f',sum(timeintervals))
end