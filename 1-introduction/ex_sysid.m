close all; clear; clc

% https://www.mathworks.com/help/ident/ug/dealing-with-multi-variable-systems-identification-and-analysis.html
load SteamEng

steam = iddata([GenVolt,Speed],[Pressure,MagVolt],0.05);
steam.InputName  = {'Pressure';'MagVolt'};
steam.OutputName = {'GenVolt';'Speed'};

% create state-space model with first half of the data
data = steam(1:250);

% visually pick the model order
nxmin = 1; nxmax = 10;
mpss = ssest(data,nxmin:nxmax);

% use transfer function method
mptf = tfest(data,4);

% to test the quality of the state-space model, simulate it on the part of
% data that was not used for estimation and compare the outputs
hf = figure; hf.Color = 'w';
compare(steam(251:450),mpss,mptf)