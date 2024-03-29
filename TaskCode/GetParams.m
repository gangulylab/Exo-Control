function Params = GetParams(Params)
% Experimental Parameters
% These parameters are meant to be changed as necessary (day-to-day,
% subject-to-subject, experiment-to-experiment)
% The parameters are all saved in 'Params.mat' for each experiment

%% Verbosity
Params.Verbose = true;

%% Experiment
Params.Task = 'Exo-Control';
switch Params.ControlMode,
    case 1, Params.ControlModeStr = 'MousePosition';
    case 2, Params.ControlModeStr = 'MouseVelocity';
    case 3, Params.ControlModeStr = 'KalmanPosVel';
    case 4, Params.ControlModeStr = 'KalmanVelocity';
end

%% Control
Params.Gain             = 1; %8;
Params.CenterReset      = false;    
Params.Assistance       = 0;      % value btw 0 and 1, 1 full assist
Params.CLDA.Type        = 3;        % 0-none, 1-refit, 2-smooth batch, 3-RML
Params.CLDA.AdaptType   = 'linear';   % {'none','linear'}, affects assistance & lambda for rml
Params.InitializationMode =4;      % 1-imagined mvmts, 2-shuffled imagined mvmts, 3-choose dir, 4-most recent KF  
Params.BaselineTime     = 0;        % secs

%% Cursor Click
Params.ClickerBins = 1;
Params.ClickingColor = [255, 255, 255];
%% Current Date and Time
% get today's date
now = datetime;
Params.YYYYMMDD = sprintf('%i',yyyymmdd(now));
Params.HHMMSS = sprintf('%02i%02i%02i',now.Hour,now.Minute,round(now.Second));

%% Data Saving

% if Subject is 'Test' or 'test' then can write over previous test
if strcmpi(Params.Subject,'Test'),
    Params.YYYYMMDD = 'YYYYMMDD';
    Params.HHMMSS = 'HHMMSS';
end

if IsWin,
    %projectdir = 'D:\new_planar\Exo-Control';
    projectdir ='C:\Users\test\Documents\GitHub\Exo-Control\';
%    projectdir = 'C:\new_planar\Exo-Control';
%    projectdir = 'D:\ganguly_planar\Exo-Control\';
%    projectdir = 'D:\2DPlanar\Exo-Control\';

elseif IsOSX,
    projectdir = '/Users/daniel/Projects/Exo-Control/';
else,
%     projectdir = '~/Projects/GangulyLab/Exo-Control/';
    projectdir = '~/Projects/Exo-Control/';
    butter(1,[.1,.5]);
end
addpath(genpath(fullfile(projectdir,'TaskCode')));
% create folders for saving
Params.ProjectDir = projectdir;
datadir = fullfile(projectdir,'Data',Params.Subject,Params.YYYYMMDD,Params.HHMMSS);
Params.Datadir = datadir;
mkdir(datadir);

%% Sync to Blackrock
Params.SerialSync = false;
Params.SyncDev = '/dev/ttyS1';
Params.BaudRate = 115200;

Params.ArduinoSync = false;

%%
Params.PlanarConnected          = 0;    % is the planar hardware connected? 
Params.skipSync                 = 0;    % Do you want to skip the VSync tests?
% Params.GenNeuralFeaturesFlag    = true; % 

%% Timing
Params.ScreenRefreshRate = 5; % Hz
Params.UpdateRate = 5; %10 = Imagined; 5 = control % Hz

%% Targets
Params.TargetSize = 30;
Params.OutTargetColor = [55,255,0];
Params.InTargetColor = [255,55,0];

Params.StartTargetPosition  = [0,0];
Params.TargetRect = ...
    [-Params.TargetSize -Params.TargetSize +Params.TargetSize +Params.TargetSize];

Params.ReachTargetAngles = (0:45:315)';
Params.ReachTargetRadius = 100;
Params.ReachTargetPositions = ...
    Params.StartTargetPosition ...
    + Params.ReachTargetRadius ...
    * [cosd(Params.ReachTargetAngles) sind(Params.ReachTargetAngles)];

Params.NumReachTargets = length(Params.ReachTargetAngles);


%% Cursor
Params.CursorColor = [95, 6, 150];
Params.CursorSize = 15;
Params.CursorRect = [-Params.CursorSize -Params.CursorSize ...
    +Params.CursorSize +Params.CursorSize];

%% Kalman Filter Properties
a = 0.825; % 0.85;%0.825;
w = 150;
G = Params.Gain;
t = 1/Params.UpdateRate;
if Params.ControlMode>=3,
    Params.KF.A = [...
        1	0	G*t	0	0;
        0	1	0	G*t	0;
        0	0	a	0	0;
        0	0	0	a	0;
        0	0	0	0	1];
    Params.KF.W = [...
        0	0	0	0	0;
        0	0	0	0	0;
        0	0	w	0	0;
        0	0	0	w	0;
        0	0	0	0	0];
    Params.KF.P = eye(5);
    Params.KF.InitializationMode = Params.InitializationMode; % 1-imagined mvmts, 2-shuffled
    if Params.ControlMode==4, % set velocity kalman filter flag
        Params.KF.VelKF = true;
    else,
        Params.KF.VelKF = false;
    end
end

%% Velocity Command Online Feedback
Params.DrawVelCommand.Flag = true;
Params.DrawVelCommand.Rect = [-425,-425,-350,-350];

%% Trial and Block Types
Params.NumImaginedBlocks    = 0;
Params.NumAdaptBlocks       = 0;
Params.NumFixedBlocks       = 1;
Params.NumTrialsPerBlock    = length(Params.ReachTargetAngles);
Params.TargetSelectionFlag  = 1; % 1-pseudorandom, 2-random
switch Params.TargetSelectionFlag,
    case 1, Params.TargetFunc = @(n) mod(randperm(n),Params.NumReachTargets)+1;
    case 2, Params.TargetFunc = @(n) mod(randi(n,1,n),Params.NumReachTargets)+1;
end

%% CLDA Parameters
TypeStrs                = {'none','refit','smooth_batch','rml'};
Params.CLDA.TypeStr     = TypeStrs{Params.CLDA.Type+1};

Params.CLDA.UpdateTime = 80; % secs, for smooth batch
Params.CLDA.Alpha = exp(log(.5) / (120/Params.CLDA.UpdateTime)); % for smooth batch

Params.CLDA.Lambda = 5000; %exp(log(.5) / (30*Params.UpdateRate)); % for RML
Params.CLDA.FinalLambda = 5000; %exp(log(.5) / (500*Params.UpdateRate));
DeltaLambda = (Params.CLDA.FinalLambda - Params.CLDA.Lambda) ...
    / ((Params.NumAdaptBlocks-2)...
    *Params.NumTrialsPerBlock...
    *Params.UpdateRate * 3); % bins/trial;
Params.CLDA.DeltaLambda = DeltaLambda; % for RML

switch Params.CLDA.AdaptType,
    case 'none',
        Params.CLDA.DeltaLambda = 0;
        Params.CLDA.DeltaAssistance = 0;
    case 'linear',
        switch Params.CLDA.Type,
            case 2, % smooth batch
                Params.CLDA.DeltaAssistance = ... % linearly decrease assistance
                    Params.Assistance...
                    /(Params.NumAdaptBlocks*Params.NumTrialsPerBlock*5/Params.CLDA.UpdateTime);
            case 3, % RML
                Params.CLDA.DeltaAssistance = ... % linearly decrease assistance
                    Params.Assistance...
                    /((Params.NumAdaptBlocks-1)*Params.NumTrialsPerBlock);
            otherwise, % none or refit
                Params.CLDA.DeltaAssistance = 0;
        end
end

%% Hold Times
Params.TargetHoldTime = 4;
Params.InterTrialInterval = 5;
Params.InstructedGraspTime = 8;
Params.InstructedDelayTime = 10;
Params.MaxStartTime = 20;
Params.MaxReachTime = 30;
Params.InterBlockInterval = 10; % 0-10s, if set to 10 use instruction screen
Params.ImaginedMvmtTime = 10;
 
%% Feedback
Params.FeedbackSound = false;
Params.ErrorWaitTime = .5;
Params.ErrorSound = 1000*audioread('buzz.wav');
Params.ErrorSoundFs = 8192;
[Params.RewardSound,Params.RewardSoundFs] = audioread('reward1.wav');
% play sounds silently once so Matlab gets used to it
sound(0*Params.ErrorSound,Params.ErrorSoundFs)

%% BlackRock Params
Params.GenNeuralFeaturesFlag =true;
Params.ZscoreRawFlag = true;
Params.UpdateChStatsFlag = false;
Params.ZscoreFeaturesFlag = true;
Params.UpdateFeatureStatsFlag = false;
Params.SaveProcessed = false;
Params.SaveRaw = true;

Params.DimRed.Flag = false;
Params.DimRed.InitMode = 2; % 1-use imagined mvmts, 2-choose dir
Params.DimRed.InitAdapt = true;
Params.DimRed.InitFixed = ~Params.DimRed.InitAdapt;
Params.DimRed.Method = 1; % 1-pca, 2-fa
Params.DimRed.AvgTrialsFlag = false; % 0-cat imagined mvmts, 1-avg imagined mvmts
Params.DimRed.NumDims = [];

Params.Fs = 1000;
Params.NumChannels = 128;
Params.BufferTime = 2; % secs longer for better phase estimation of low frqs
Params.BufferSamps = Params.BufferTime * Params.Fs;
Params.BadChannels = [];
RefModeStr = {'none','common_mean','common_median'};
Params.ReferenceMode = 2; % 0-no ref, 1-common mean, 2-common median
Params.ReferenceModeStr = RefModeStr{Params.ReferenceMode+1};

% filter bank - each element is a filter bank
% fpass - bandpass cutoff freqs
% feature - # of feature (can have multiple filters for a single feature
% eg., high gamma is composed of multiple freqs)
Params.FilterBank = [];
Params.FilterBank(end+1).fpass = [.5,4];    % delta
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = true;
Params.FilterBank(end).feature = 2;

Params.FilterBank(end+1).fpass = [4,8];     % theta
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 3;

Params.FilterBank(end+1).fpass = [8,13];    % alpha
Params.FilterBank(end).buffer_flag = true;
Params.FilterBank(end).hilbert_flag = true;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 4;

Params.FilterBank(end+1).fpass = [13,19];   % beta1
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 5;

Params.FilterBank(end+1).fpass = [19,30];   % beta2
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 5;

Params.FilterBank(end+1).fpass = [30,36];   % low gamma1 
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 6;

Params.FilterBank(end+1).fpass = [36,42];   % low gamma2 
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 6;

Params.FilterBank(end+1).fpass = [42,50];   % low gamma3
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 6;

Params.FilterBank(end+1).fpass = [70,77];   % high gamma1
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [77,85];   % high gamma2
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [85,93];   % high gamma3
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [93,102];  % high gamma4
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [102,113]; % high gamma5
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [113,124]; % high gamma6
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [124,136]; % high gamma7
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

Params.FilterBank(end+1).fpass = [136,150]; % high gamma8
Params.FilterBank(end).buffer_flag = false;
Params.FilterBank(end).hilbert_flag = false;
Params.FilterBank(end).phase_flag = false;
Params.FilterBank(end).feature = 7;

% compute filter coefficients
for i=1:length(Params.FilterBank),
    [b,a] = butter(3,Params.FilterBank(i).fpass/(Params.Fs/2));
    Params.FilterBank(i).b = b;
    Params.FilterBank(i).a = a;
end

% unique pwr feature + all phase features
Params.NumBuffer = sum([Params.FilterBank.buffer_flag]);
Params.NumHilbert = sum([Params.FilterBank.hilbert_flag]);
Params.NumPhase = sum([Params.FilterBank.phase_flag]);
Params.NumPower = length(unique([Params.FilterBank.feature]));
Params.NumFeatures = Params.NumPower + Params.NumPhase;

%% Save Parameters
save(fullfile(Params.Datadir,'Params.mat'),'Params');

end % GetParams

