function debug_Place(place)
    if 0 ~= sum(place==0)
        fprintf('ESKALATION/ERROR in place/spline');
        fprintf('place(%d) [%s] \n',length(place),join(string( place ), ','));
    end
end

