function rotationsmatrix = RotationDegUmZ(alpha)

    alpha = deg2rad(alpha);

    rotationsmatrix = [cos(alpha) -sin(alpha)  0;
                       sin(alpha)  cos(alpha)  0;
                       0           0           1];
end