
clc 
clear all
close


KSetUp;
base_points20 = importdata('20_Punkte_time_angle.txt');




timeintervals = base_points20 (1:size(base_points20 ,1)-1,1);
timeintervals(:,1) = 0.1;

base_points = base_points20 (:,2:size(base_points20 ,2));
base_points = deg2rad(base_points);



% Zulässige Werte für Winkel, Geschwindigkeit, Beschleunigung, Huk
% Winkel
max_jointangle = deg2rad([185,14,144,350,120,350]);
min_jointangle = deg2rad([-185,-130,-100,-350,-120,-350]);
% Geschwindigkeit
max_velocity = deg2rad([400,400,400,400,400,400]);
min_velocity = deg2rad([-400,-400,-400,-400,-400,-400]);
% Beschleunigung
max_acceleration = deg2rad([200,200,200,200,200,200]);
min_acceleration = deg2rad([-200,-200,-200,-200,-200,-200]);
% Huk
max_jerk = deg2rad([1000,1000,1000,1000,1000,1000]);
min_jerk = deg2rad([-1000,-1000,-1000,-1000,-1000,-1000]);

figure;
for i=1:size(base_points,2)
    fprintf('SPLINE %d',i);
    [t,td,tdd,tddd,time,place] = spline(base_points(:,i)',timeintervals,false);
    
    subplot(4,size(base_points,2),1+(i-1))
    hold on
    plot(compute_time(timeintervals),base_points(:,i), "linestyle", "none","marker","o");
    plot(time,t, "Color","green");
   
   
    title("Spline ",i);
    xlabel("Zeit [s]");
    ylabel("Winkel [rad]");

   
    y_limit = [min_jointangle(i)-0.4   max_jointangle(i)+0.4];
    yline(min_jointangle(i),'--',"minimale zulässige Winkel");
    yline(max_jointangle(i),'--',"maximale zulässige Winkel");


    txt_y_min = "min:" + min(base_points(:,i)) ;
    txt_y_max = "max:" + max(base_points(:,i)) ;
    yline(max(base_points(:,i)),'-',txt_y_max);
    yl_min= yline(min(base_points(:,i)),'-',txt_y_min);
    yl_min.LabelVerticalAlignment = 'bottom';
    ylim(y_limit);
    
    ax = gca;
    ax.YAxis.Exponent = 0;
    hold off
 
    %Plotten der 1. Ableitung/Geschwindigkeit
    subplot(4,size(base_points,2),7+(i-1))
    plot(time,td, "Color","magenta");
    title("Geschindigkeit");
    xlabel("Zeit [s]");
    ylabel("Geschindigkeit [rad/s]");
    y_limit = [min_velocity(i)-1   max_velocity(i)+1];
    yline(min_velocity(i),'--',"minimale zulässige Geschindigkeit");
    yline(max_velocity(i),'--',"maximale zulässige Geschindigkeit");
    txt_y_min = "min:" + min(td(:,i)) ;
    txt_y_max = "max:" + max(td(:,i)) ;
    yline(max(td),'-', txt_y_max);
    yl_min = yline(min(td),'-', txt_y_min);
    yl_min.LabelVerticalAlignment = 'bottom';
    ylim(y_limit);
    ax = gca;
    ax.YAxis.Exponent = 0;
    
    %Plotten der 2. Ableitung/Beschleunigung 
    subplot(4,size(base_points,2),13+(i-1))
    hold on 
    plot(time,tdd, "Color","yellow");
    title("Bechleunigung");
    xlabel("Zeit [s]");
    ylabel("Bechleunigung [rad/s^2]");
    y_limit = [min_acceleration(i)-4   max_acceleration(i)+4];
    yline(min_velocity(i),'--',"minimale zulässige Bechleunigung");
    yline(max_velocity(i),'--',"maximale zulässige Bechleunigung");
    txt_y_min = "min:" + min(tdd) ;
    txt_y_max = "max:" + max(tdd) ;
    yline(max(tdd),'-', txt_y_max);
  
  
    yl_min  = yline(min(tdd),'-', txt_y_min);
    yl_min.LabelVerticalAlignment = 'bottom';
   
    ylim(y_limit);
    %Plottet nur die Ableitungen der Punkte 
    %plot(ti,[0,ableitungen2,0],"marker","o");
    ax = gca;
    ax.YAxis.Exponent = 0;
    hold off 
    
    %Plotten der 3. Ableitung/Ruck
    subplot(4,size(base_points,2),19+(i-1)) 
    plot(time,tddd, "Color","red");
    title("Huk");
    xlabel("Zeit [s]");
    ylabel("Huk [rad/s^3] ");
    y_limit = [min_jerk(i)-50   max_jerk(i)+50];
    yline(min_velocity(i),'--',"minimale zulässige Huk");
    yl_min= yline(max_velocity(i),'--',"maximale zulässige Huk");
    yl_min.LabelVerticalAlignment = 'bottom';
    yl_min.LabelHorizontalAlignment = 'left';
    txt_y_min = "min:" + min(tddd) ;
    txt_y_max = "max:" + max(tddd) ;
    yline(max(tddd),'-', txt_y_max);
    yl_min = yline(min(tddd),'-', txt_y_min);
    yl_min.LabelVerticalAlignment = 'bottom';
    ylim(y_limit);
    ax = gca;
    ax.YAxis.Exponent = 0;

end
fprintf('Zeit: %f',sum(timeintervals))






function array = compute_time(intervals)
array(1)=0;
for i=2:length(intervals)+1;
    array(i)= sum(intervals(1:i-1));
end
end






