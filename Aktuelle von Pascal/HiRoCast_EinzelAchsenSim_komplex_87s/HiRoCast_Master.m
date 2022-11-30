%% Start simulation environment
clc
clear
close all
memory
a = rand(20000);
memory

%% Definition and load of simulation parameters
modelName = "KR360_SimpleLadleOne";
pathDirRoboModel = 'C:\Users\penczek\Desktop\pathDirRoboModel\20211209_HiroCast_KR60.rdk'; %modelName + "\"
pathDirSimData = '\\wsl$\Ubuntu\home\penczek\FromCylinderToKelleV7\sloshingCylinderFrom8\';

%%TCPIP Setup (Used to indicate that files have arrived in fluid folder)
%%socket = tcpip('0.0.0.0',49999,'NetworkRole','server');
%%socket.InputBufferSize = 10000;
%%fopen(socket);

% Check if ubuntu is running
%[status,result] = system('tasklist /FI "imagename eq ubuntu.exe" /fo table /nh');
%result(1:1) = []; 
%if startsWith(result , 'ubuntu.exe') == 1
%else
 %   system('C:/Users/penczek/AppData/Local/Microsoft/WindowsApps/ubuntu.exe &');
%end

[foundModel, maximumAxleSpeed, maximumAxleAcceleration, maximumTCPSpeed, ...
    maximumTCPAcceleration, nameRoboModel, ...
    fluidSimreorientationBoundaries, fluidSimladleOrientationDiscretization, fluidSimStartTime, fluidSimEndTime, fluidSimWriteInterval, fluidSimAcceleration, ...
    robotStartAndEndStep, robotReorientationBoundaries, robotReorientStepSize, robotTunnelDiameter, robotRingDistanceOfPoints, ...
    fluidMaxresultFluidVektor, fluidStableVectorLimit, fluidGainFactorForSettingTheAcceleration, fluidMinHeightCutPoints, fluidHeightMax, fluidAlphaWaterAtMax, fluidMaxWaterSurfaceDeflection, fluidSamplingRateOfFluidSimulationResults, fluidTrendDetectionByNumberOfSteps ...
     deletingFluidSimFiles, ladleIsRotationallySymmetrical, simStableStatefluid, generateRoboPoints, copySimFiles, deleteUnnecessaryThingsFromCellCenters, plotSimResults ...
    ] = HiRoCast_Parameters(modelName);

%% Generating the stable states at Ladle orientations (fluid simulation, Possibility to paralelize with pose generation)
if simStableStatefluid
    
%Send ModelName to the Ubuntu version so it can start the Simulation
sendModelNameToLinux(modelName);

%Until Results has been sent... wait for Results
while isempty(dir('C:\Users\penczek\Desktop\pathDirRoboModel\results\fluid\*.txt'))

end

end
%% Generate valid robot poses (Possibility to paralelize the individual sections)
%if generateRoboPoints
HiRoCast_RoboDK_API(robotStartAndEndStep, robotReorientationBoundaries, robotReorientStepSize, robotTunnelDiameter, robotRingDistanceOfPoints, pathDirRoboModel); % TODO: paralelisieren der Simulation
%end

%% Optimal control
HiRoCast_OptimalControl();

%% Validation and optimization loops
HiRoCast_Validation();
HiRoCast_OptimizationLoop();

%% Generate KRL Code
HiRoCast_GenerateKRLCode();