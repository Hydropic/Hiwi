function [] = HiRoCast_GenerateKRLCode(path)
% Show robot code in: Kuka robot language 
% Generate.EMI (TODO: data format, time in ms)

filename= 'Emily1_AXIAL.EMI';
%formatSpec = '  %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f\n';
file_ID = fopen(filename,'w');

%Header
fprintf(file_ID,'[HEADER]\n');
fprintf(file_ID,'  GEAR_NOMINAL_VEL = %.6f\n',1);
fprintf(file_ID,'  CRC = %d\n',420);

%Data (array in ms)
path = path(1:12:end,:);
fprintf(file_ID,'[RECORDS]\n');
for n = 1 : size(path,1)
  fprintf(file_ID,'  %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f\n',path(n,:));
end

%End
fprintf(file_ID,'[END]\n');
fclose(file_ID);
end

