function rotationsmatrix = RotationDegUmX(gamma)

    gamma = deg2rad(gamma);
    
    rotationsmatrix = [1    0                     0;
                       0    cos(gamma)  -sin(gamma);
                       0    sin(gamma)   cos(gamma)];
end