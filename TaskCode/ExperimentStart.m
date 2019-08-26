 function ExperimentStart(Subject,ControlMode,BLACKROCK,DEBUG)
% ExperimentStart('Test',2,0,0)
% ExperimentStart('bravo1',4,1,0)
% function ExperimentStart(Subject,ControlMode)
% Subject - string for the subject id
% ControlMode - [1,2,3,4] for mouse pos, mouse vel, pos/vel kalman, vel
%   kalman
% BLACKROCK - [0,1] if 1, collects, processes, and saves neural data
% DEBUG - [0,1] if 1, enters DEBUG mode in which screen is small and cursor
%   remains unhidden

%% Clear All and Close All
clearvars -except Subject ControlMode BLACKROCK DEBUG
clearvars -global
clc
warning off

if ~exist('Subject','var'), Subject = 'Test'; DEBUG = 1; end
if ~exist('ControlMode','var'), ControlMode = 2; end
if ~exist('BLACKROCK','var'), BLACKROCK = 0; end
if ~exist('DEBUG','var'), DEBUG = 0; end

AssertOpenGL;
KbName('UnifyKeyNames');

if strcmpi(Subject,'Test'), Subject = 'Test'; end

%% Retrieve Parameters from Params File
Params.Subject = Subject;
Params.ControlMode = ControlMode;
Params.BLACKROCK = BLACKROCK;
Params.DEBUG = DEBUG;
Params = GetParams(Params);

%% Initialize Blackrock System
if BLACKROCK,
    if IsWin,
        addpath('C:\Program Files (x86)\Blackrock Microsystems\Cerebus Windows Suite')
    elseif IsLinux,
        addpath('/usr/local/CereLink');
    end
    cbmex('close'); % always close
    cbmex('open'); % open library
    cbmex('trialconfig', 1); % empty the buffer
end

%% Initialize Sync to Blackrock
if Params.SerialSync,
    Params.SerialPtr = serial(Params.SyncDev, 'BaudRate', Params.BaudRate);
    fopen(Params.SerialPtr);
    fprintf(Params.SerialPtr, '%s\n', 'START');
end
switch Params.PlanarConnected
    
    case 0
        Params.Arduino.devBBS = nan;
    case 1
        % Params.ArduinoPtr = arduino('COM41','Due','Libraries','I2C');   % Planar Laptop
        % Params.ArduinoPtr = arduino('COM9','Due','Libraries','I2C');  % Rob's Laptop
        Params.ArduinoPtr = arduino('/dev/ttyACM0','Due','Libraries','I2C');
        Params.ArduinoPin = 'D13';
        writeDigitalPin(Params.ArduinoPtr, Params.ArduinoPin, 0); % make sure the pin is at 0
        PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,20);
        
        Params.Arduino.devBBS   = i2cdev(Params.ArduinoPtr,'0x01','bus',0); % BBS is on Brain Box bus I2C0, device 1
end

    Params.Arduino.planar.velParams.minSpeed     = -100; % mm/s
    Params.Arduino.planar.velParams.maxSpeed     =  100; % mm/s
    Params.Arduino.planar.velParams.bits         =  12; % bits
    Params.Arduino.planar.velParams.f_speed2bits      = @(speed) round(((speed-Params.Arduino.planar.velParams.minSpeed)...
                                                            ./(Params.Arduino.planar.velParams.maxSpeed-Params.Arduino.planar.velParams.minSpeed))...
                                                            .*(2^Params.Arduino.planar.velParams.bits-1));

	Params.Arduino.planar.posParams.screenResolution  = [1920,1080];  % Fancy B1 Monitor
% 	Params.Arduino.planar.posParams.screenResolution  = [1680,1050];  % Monitor in 133SDH
% 	Params.Arduino.planar.posParams.screenResolution  = [2880,1620];  % Rob's laptop
% 	Params.Arduino.planar.posParams.screenResolution  = [1920,1200];  % Leeann/Star Monitor in 133SDH
% 	Params.Arduino.planar.posParams.screenResolution  = [1600,900];   % Labtop
    Params.Arduino.planar.posParams.planarBounds      = [-300,300,-10,300];   % mm
    Params.Arduino.planar.posParams.planarPlotLoc     = [Params.Arduino.planar.posParams.screenResolution(1)/2,...
                                                            Params.Arduino.planar.posParams.screenResolution(2)/2+...
                                                            -(Params.Arduino.planar.posParams.planarBounds(4)-Params.Arduino.planar.posParams.planarBounds(3))/2];           % mm
    Params.Arduino.planar.pos   = [0;0];            % mm
    
    Params.Arduino.planar.usePlanarAsCursor     = 1;
    
    Params.Arduino.planar.posParams.minPos            = -300; % mm
    Params.Arduino.planar.posParams.maxPos            =  300; % mm
    Params.Arduino.planar.posParams.bits              =  12; % bits
    Params.Arduino.planar.posParams.f_bits2pos        = @(bits) ((double(bits)./(2^Params.Arduino.planar.posParams.bits-1))...
                                                            .*(Params.Arduino.planar.posParams.maxPos-Params.Arduino.planar.posParams.minPos)...
                                                            +Params.Arduino.planar.posParams.minPos);
    
    Params.Arduino.planar.enable                = 0;    % 1-bit     Turn planar off and on
    Params.Arduino.planar.velocityMode          = 0;    % 1-bit     Move to target, or accept sent velocities
    Params.Arduino.planar.target                = 0;    % 4-bits    For 16 preset targets
    Params.Arduino.planar.vel   = [0;0];
    
    
    Params.Arduino.glove.enable                = 0;    % 1-bit     Turn planar off and on
    Params.Arduino.glove.admittanceMode        = 0;    % 1-bit     Move to target, or accept sent velocities
    Params.Arduino.glove.target                = 0;    % 4-bits    For 16 preset targets
    
    Params.Arduino  = UpdateArduino(Params.Arduino);    


%% Neural Signal Processing
% create neuro structure for keeping track of all neuro updates/state
% changes
Neuro.ZscoreRawFlag     = Params.ZscoreRawFlag;
Neuro.ZscoreFeaturesFlag= Params.ZscoreFeaturesFlag;
Neuro.DimRed            = Params.DimRed;
Neuro.CLDA              = Params.CLDA;
Neuro.SaveProcessed     = Params.SaveProcessed;
Neuro.SaveRaw           = Params.SaveRaw;
Neuro.FilterBank        = Params.FilterBank;
Neuro.NumChannels       = Params.NumChannels;
Neuro.BufferSamps       = Params.BufferSamps;
Neuro.BadChannels       = Params.BadChannels;
Neuro.ReferenceMode     = Params.ReferenceMode;
Neuro.NumPhase          = Params.NumPhase;
Neuro.NumPower          = Params.NumPower;
Neuro.NumBuffer         = Params.NumBuffer;
Neuro.NumHilbert        = Params.NumHilbert;
Neuro.NumFeatures       = Params.NumFeatures;
Neuro.LastUpdateTime    = GetSecs;
Neuro.UpdateChStatsFlag = Params.UpdateChStatsFlag;
Neuro.UpdateFeatureStatsFlag = Params.UpdateFeatureStatsFlag;

% initialize filter bank state
for i=1:length(Params.FilterBank),
    Neuro.FilterBank(i).state = [];
end

% initialize stats for each channel for z-scoring
Neuro.ChStats.wSum1  = 0; % count
Neuro.ChStats.wSum2  = 0; % squared count
Neuro.ChStats.mean   = zeros(1,Params.NumChannels); % estimate of mean for each channel
Neuro.ChStats.S      = zeros(1,Params.NumChannels); % aggregate deviation from estimated mean for each channel
Neuro.ChStats.var    = zeros(1,Params.NumChannels); % estimate of variance for each channel

% initialize stats for each feature for z-scoring
Neuro.FeatureStats.wSum1  = 0; % count
Neuro.FeatureStats.wSum2  = 0; % squared count
Neuro.FeatureStats.mean   = zeros(1,Params.NumFeatures*Params.NumChannels); % estimate of mean for each channel
Neuro.FeatureStats.S      = zeros(1,Params.NumFeatures*Params.NumChannels); % aggregate deviation from estimated mean for each channel
Neuro.FeatureStats.var    = zeros(1,Params.NumFeatures*Params.NumChannels); % estimate of variance for each channel

% create low freq buffers
Neuro.FilterDataBuf = zeros(Neuro.BufferSamps,Neuro.NumChannels,Neuro.NumBuffer);

%% Kalman Filter
if Params.ControlMode>=3,
    KF = Params.KF;
    KF.CLDA = Params.CLDA;
    KF.Lambda = Params.CLDA.Lambda;
else,
    KF = [];
end

%% Load Clicker
f = load(fullfile('clicker','clicker_svm_mdl.mat'));
Clicker.Model = f.model;
Clicker.Func = @ (X) click_classifier(X,Clicker.Model);

%% Check Important Params with User
LogicalStr = {'off', 'on'};
IMStr = {'imagined mvmts', 'shuffled imagined mvmts', 'prev mvmts', 'prev adapted'};
DimRedStr = {'PCA', 'FA'};

fprintf('\n\nImportant Experimental Parameters:')
fprintf('\n\n  Task Parameters:')
fprintf('\n    - task: %s', Params.Task)
fprintf('\n    - subject: %s', Params.Subject)
fprintf('\n    - control mode: %s', Params.ControlModeStr)
fprintf('\n    - blackrock mode: %s', LogicalStr{Params.BLACKROCK+1})
fprintf('\n    - debug mode: %s', LogicalStr{Params.DEBUG+1})
fprintf('\n    - serial sync: %s', LogicalStr{Params.SerialSync+1})
fprintf('\n    - arduino sync: %s', LogicalStr{Params.ArduinoSync+1})

fprintf('\n\n  Neuro Processing Pipeline:')
if Params.GenNeuralFeaturesFlag,
    fprintf('\n    - generating neural features!')
else,
    fprintf('\n    - reference mode: %s', Params.ReferenceModeStr)
    fprintf('\n    - zscore raw: %s', LogicalStr{Params.ZscoreRawFlag+1})
    fprintf('\n    - zscore features: %s', LogicalStr{Params.ZscoreFeaturesFlag+1})
    fprintf('\n    - save raw data: %s', LogicalStr{Params.SaveRaw+1})
    fprintf('\n    - save filtered data: %s', LogicalStr{Params.SaveProcessed+1})
end
fprintf('\n    - dimensionality reduction: %s', LogicalStr{Params.DimRed.Flag+1})
if Params.DimRed.Flag,
    fprintf('\n      - method: %s', DimRedStr{Params.DimRed.Method})
    fprintf('\n      - before clda: %s', LogicalStr{Params.DimRed.InitAdapt+1})
    fprintf('\n      - before fixed: %s', LogicalStr{Params.DimRed.InitFixed+1})
end

fprintf('\n\n  BCI Parameters:')
fprintf('\n    - Imagined Movements: %s', LogicalStr{double(Params.NumImaginedBlocks>0) +1})
fprintf('\n      - initialization mode: %s', IMStr{Params.InitializationMode})
fprintf('\n    - Adaptation Decoding: %s', LogicalStr{double(Params.NumAdaptBlocks>0) +1})
if Params.NumAdaptBlocks>0,
    fprintf('\n      - adapt type: %s', Params.CLDA.TypeStr)
    fprintf('\n      - adapt change type: %s', Params.CLDA.AdaptType)
end
fprintf('\n    - Fixed Decoding: %s', LogicalStr{double(Params.NumFixedBlocks>0) +1})


str = input('\n\nContinue? (''n'' to quit, otherwise continue)\n' ,'s');
if strcmpi(str,'n'),
    fprintf('\n\nExperiment Ended\n\n')
    return
end

%% Initialize Window
if Params.skipSync == 1
Screen('Preference', 'SkipSyncTests', 1);
end
if DEBUG
    [Params.WPTR, Params.ScreenRectangle] = Screen('OpenWindow', 0, 0, [50 50 1000 1000]);
else
    [Params.WPTR, Params.ScreenRectangle] = Screen('OpenWindow', max(Screen('Screens')), 0);
end
Params.Center = [mean(Params.ScreenRectangle([1,3])),mean(Params.ScreenRectangle([2,4]))];

% Font
Screen('TextFont',Params.WPTR, 'Arial');
Screen('TextSize',Params.WPTR, 28);

%% Start
try
    % Baseline 
    if Params.BaselineTime>0,
        % turn on update stats flags
        Neuro.UpdateChStatsFlag = true;
        Neuro.UpdateFeatureStatsFlag = true;
        
        % collect data during baseline period
        Neuro = RunBaseline(Params,Neuro);
        
        % set flags back to original vals
        Neuro.UpdateChStatsFlag = Params.UpdateChStatsFlag;
        Neuro.UpdateFeatureStatsFlag = Params.UpdateFeatureStatsFlag;
        
        % save of useful stats and params
        ch_stats = Neuro.ChStats;
        save(fullfile(Params.ProjectDir,'TaskCode','persistence','ch_stats.mat'),...
            'ch_stats','-v7.3','-nocompression');
        feature_stats = Neuro.FeatureStats;
        save(fullfile(Params.ProjectDir,'TaskCode','persistence','feature_stats.mat'),...
            'feature_stats','-v7.3','-nocompression');
    else, % if baseline is set to 0, just load stats
        f=load(fullfile(Params.ProjectDir,'TaskCode','persistence','ch_stats.mat'));
        Neuro.ChStats = f.ch_stats;
        f=load(fullfile(Params.ProjectDir,'TaskCode','persistence','feature_stats.mat'));
        Neuro.FeatureStats = f.feature_stats;
        clear('f');
    end
    
    % Imagined Cursor Movements Loop
    if Params.NumImaginedBlocks>0,
        [Neuro,KF,Params,Clicker] = RunTask(Params,Neuro,1,KF,Clicker);
    end
    
    % Adaptation Loop
    if Params.NumAdaptBlocks>0,
        [Neuro,KF,Params,Clicker] = RunTask(Params,Neuro,2,KF,Clicker);
    end
    
    % Fixed Decoder Loop
    if Params.NumFixedBlocks>0,
        [Neuro,KF,Params,Clicker] = RunTask(Params,Neuro,3,KF,Clicker);
    end
    
    
    % Save persistence folder with Data
    %mkdir(fullfile(Params.Datadir,'persistence'))
    %copyfile(fullfile(Params.ProjectDir,'TaskCode','persistence','kf_params.mat'),...
     %   fullfile(Params.Datadir,'persistence','kf_params.mat'));
    
    
    % Pause and Finish!
    ExperimentStop([],Params);
    
catch ME, % handle errors gracefully
    Screen('CloseAll')
    for i=length(ME.stack):-1:1,
        if i==1,
            errorMessage = sprintf('Error in function %s() at line %d.\n\nError Message:\n%s\n\n', ...
                ME.stack(1).name, ME.stack(1).line, ME.message);
        else,
            errorMessage = sprintf('Error in function %s() at line %d.\n\n', ...
                ME.stack(i).name, ME.stack(i).line);
        end
        fprintf(1,'\n%s\n', errorMessage);
    end
    
    keyboard;
end

end % ExperimentStart
