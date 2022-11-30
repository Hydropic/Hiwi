function [result] = loadModel(table)
varList = evalin('base','who');

if ~any(strcmp(varList,'regressionKat.mat'))
load("regressionKat.mat")
end
result = regressionKat.predictFcn(table);

end