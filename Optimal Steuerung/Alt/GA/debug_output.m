function debug_output(i,rx, ry, rz, start_z, acceleration, direction, simulation_data, variance, variance_dir, variance_a)
    % Debug output    
    fprintf('Iter. %d X:%f Y:%f Z:%f [Z:%f] A:%f dx: %f dy: %f dz: %f\n',i,rx,ry,rz,start_z,acceleration,direction(1),direction(2),direction(3));
    
    % Printing entries with valid rotation
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance);
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('Rotation: [%s] \n',debug_string);
    end
    
    % Printing entries with valid direction(acceleration) %
    % acceleration
    debug_index = (abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('Direction: [%s] \n',debug_string);
    end
    
    % Printing entries with valid rotation & direction(acceleration)
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance & ...
        abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir &...
        abs(direction(3) - simulation_data.directionZ) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('#Rotation+Direction: [%s] \n',debug_string);
    end
    
    % Printing entries with valid rotation & direction(acceleration) %
    % acceleration
    debug_index = (abs(simulation_data.rotationX - rx) <= variance & ...
        abs(simulation_data.rotationY - ry) <= variance & ...
        abs(acceleration - simulation_data.acceleration) <= variance_a & ...
        abs(direction(1) - simulation_data.directionX) <= variance_dir &...
        abs(direction(2) - simulation_data.directionY) <= variance_dir &...
        abs(direction(3) - simulation_data.directionZ) <= variance_dir); 
    debug_string = join(string( find(debug_index)  ), ',');
    
    if ismissing(debug_string) == false 
        fprintf('##Alles [%s] \n',debug_string);
    end
end

