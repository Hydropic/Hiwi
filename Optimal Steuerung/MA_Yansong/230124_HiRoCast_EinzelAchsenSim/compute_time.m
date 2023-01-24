function array = compute_time(intervals)
    array(1)=0;
    for i=2:length(intervals)+1
        array(i)= sum(intervals(1:i-1));
    end
end

