function [valuesOfAngles] = compressAngleValues(anglesArray, limit)

    %%Compress Angle Values so that they stay under a certain number
    factor = 1;
    anglesArrayCopy = anglesArray;
    anglesArrayCopy = anglesArrayCopy * factor
    while (max(anglesArrayCopy) > limit | min(anglesArrayCopy) < -limit)
        anglesArrayCopy = anglesArray;
        factor = factor - 0.01;
        anglesArrayCopy = anglesArrayCopy * factor;
    end
    maxValue = max(anglesArrayCopy);
    minValue = min(anglesArrayCopy);
    valuesOfAngles = anglesArrayCopy;
    
end