close all;
clear all;

dataXLSX = readcell('input/KomplexeOptiBahnMessung.xlsx');

optimized_translational_values_sameDistances(:, 1) = dataXLSX(:,1);
optimized_translational_values_sameDistances(:, 2) = dataXLSX(:,2);
optimized_translational_values_sameDistances(:, 3) = dataXLSX(:,3);

[aa,bb] = size(optimized_translational_values_sameDistances);

for k = 1:aa
    isnum(k,1) = isnumeric(optimized_translational_values_sameDistances{k,1})
end

for s = 1:aa
    if isnum(s,1) == 1
        optimized_translational_values_sameDistances_isnum(s, 1) = cell2mat(optimized_translational_values_sameDistances(s, 1))
        optimized_translational_values_sameDistances_isnum(s, 2) = cell2mat(optimized_translational_values_sameDistances(s, 2))
        optimized_translational_values_sameDistances_isnum(s, 3) = cell2mat(optimized_translational_values_sameDistances(s, 3))
    else
    end
end

%Calculate distance between timeintervals
timeintervals = zeros(1,size(optimized_translational_values_sameDistances_isnum(:, 1),1) - 1);
sizeofArray = length(optimized_translational_values_sameDistances_isnum(:, 1)) - 1;
disp(sizeofArray)
for i = 1:sizeofArray
    timeintervals(i) = optimized_translational_values_sameDistances_isnum(i+1, 1) - optimized_translational_values_sameDistances_isnum(i, 1);
end

%Configure Figure and plot
fig = figure(1);
subplot(2,1,1)
[spline, velocity, acceleration, ruck , time] =  splineOptimal(optimized_translational_values_sameDistances_isnum(:, 2),timeintervals,false);
plot(time, spline)
title('X Angle (Swap)')
xlabel 'Timestep';
ylabel 'XAngle'

subplot(2,1,2)
[spline2, velocity, acceleration, ruck , time2] =  splineOptimal(optimized_translational_values_sameDistances_isnum(:, 3),timeintervals,false);
plot(time2, spline2)
title('Y Angle (Swap)')
xlabel 'Timestep';
ylabel 'YAngle'

saveas(fig,'SwapAngle.jpg' )
    
