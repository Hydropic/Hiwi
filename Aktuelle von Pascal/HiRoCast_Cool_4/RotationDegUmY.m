function rotationsmatrix = RotationDegUmY(beta)

    beta = deg2rad(beta);

    rotationsmatrix = [cos(beta)  0 sin(beta);
                       0          1         0;
                       -sin(beta) 0 cos(beta)];
end