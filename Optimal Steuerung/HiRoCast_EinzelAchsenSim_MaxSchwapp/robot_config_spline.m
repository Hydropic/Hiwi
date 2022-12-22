function configurations = robot_config_spline(output,schritte)
timeintervals = output(1:size(output,1)-1,1);
base_points = output(:,2:size(output,2));
 
[t1,td1,tdd1,tddd1,time1,place1] = spline(base_points(:,1),timeintervals,false,schritte);
[t2,td2,tdd2,tddd2,time2,place2] = spline(base_points(:,2),timeintervals,false,schritte);
[t3,td3,tdd3,tddd3,time3,place3] = spline(base_points(:,3),timeintervals,false,schritte);
[t4,td4,tdd4,tddd4,time4,place4] = spline(base_points(:,4),timeintervals,false,schritte);
[t5,td5,tdd5,tddd5,time5,place5] = spline(base_points(:,5),timeintervals,false,schritte);
[t6,td6,tdd6,tddd6,time6,place6] = spline(base_points(:,6),timeintervals,false,schritte);

configurations=[];
for i=1:length(t1)
    configurations(i,1:6)= [t1(i), t2(i), t3(i), t4(i), t5(i), t6(i)];
end
end