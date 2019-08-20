function [Neuro,KF,Params,Clicker] = RunLoop(Params,Neuro,TaskFlag,DataDir,KF,Clicker)
% Defines the structure of collected data on each trial
% Loops through blocks and trials within blocks

global Cursor

%% Start Experiment
DataFields = struct(...
    'Params',Params,...
    'Block',NaN,...
    'Trial',NaN,...
    'TrialStartTime',NaN,...
    'TrialEndTime',NaN,...
    'TargetID',NaN,...
    'MvmtAxisAngle',NaN,...
    'TargetPosition',NaN,...
    'Time',[],...
    'CursorAssist',[],...
    'CursorState',[],...
    'ClickerState',[],...
    'PlanarState',[],...
    'IntendedCursorState',[],...
    'KalmanFilter',[],...
    'NeuralTime',[],...
    'NeuralTimeBR',[],...
    'NeuralSamps',[],...
    'NeuralFeatures',{{}},...
    'NeuralFactors',{{}},...
    'BroadbandData',{{}},...
    'ProcessedData',{{}},...
    'ChStats',[],...
    'FeatureStats',[],...
    'ErrorID',0,...
    'ErrorStr','',...
    'Events',[]...
    );

switch TaskFlag,
    case 1, NumBlocks = Params.NumImaginedBlocks;
    case 2, NumBlocks = Params.NumAdaptBlocks;
    case 3, NumBlocks = Params.NumFixedBlocks;
end

%%  Loop Through Blocks of Trials
Trial = 0;
TrialBatch = {};
tlast = GetSecs;
Cursor.LastPredictTime = tlast;
Cursor.LastUpdateTime = tlast;
for Block=1:NumBlocks, % Block Loop

    % random order of reach targets for each block
    TargetOrder = Params.TargetFunc(Params.NumTrialsPerBlock);
    Cursor.State = [0,0,1]';
    Cursor.ClickState=0;
    Cursor.IntendedState = [0,0,1]';
    Cursor.Vcommand = [0]';

    for TrialPerBlock=1:Params.NumTrialsPerBlock, % Trial Loop
        % if smooth batch on & enough time has passed, update KF btw trials
        if TaskFlag==2 && Neuro.CLDA.Type==2,
            TrialBatch{end+1} = sprintf('Data%04i.mat', Trial);
            if (GetSecs-tlast)>Neuro.CLDA.UpdateTime,
                Neuro.KF.CLDA = Params.CLDA;
                if Neuro.DimRed.Flag,
                    KF = FitKF(Params,fullfile(Params.Datadir,'BCI_CLDA'),2,...
                        KF,TrialBatch,Neuro.DimRed.F);
                else,
                    KF = FitKF(Params,fullfile(Params.Datadir,'BCI_CLDA'),2,...
                        KF,TrialBatch);
                end
                tlast = GetSecs;
                TrialBatch = {};
                % decrease assistance after batch update
                if Cursor.Assistance>0,
                    Cursor.Assistance = Cursor.Assistance - Cursor.DeltaAssistance;
                    Cursor.Assistance = max([Cursor.Assistance,0]);
                end
            end
        elseif TaskFlag==2 && Neuro.CLDA.Type==3,
            % decrease assistance after batch update
            if Cursor.Assistance>0,
                Cursor.Assistance = Cursor.Assistance - Cursor.DeltaAssistance;
                Cursor.Assistance = max([Cursor.Assistance,0]);
            end
        end
        
        % update trial
        Trial = Trial + 1;
        TrialIdx = TargetOrder(TrialPerBlock);
        
        % set up trial
        TrialData = DataFields;
        TrialData.Block = Block;
        TrialData.Trial = Trial;
        TrialData.TargetID = TrialIdx;
        TrialData.MvmtAxisAngle = Params.MvmtAxisAngle;
        TrialData.TargetPosition = Params.ReachTargetPositions(TrialIdx);
        TrialData.KalmanFilter = KF;
        TrialData.ChStats = Neuro.ChStats;
        TrialData.FeatureStats = Neuro.FeatureStats;

        % Run Trial
        TrialData.TrialStartTime  = GetSecs;
        if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'TST'); end
        [TrialData,Neuro,KF,Params,Clicker] = RunTrial(TrialData,Params,Neuro,TaskFlag,KF,Clicker);
        if Params.SerialSync, fprintf(Params.SerialPtr, '%s\n', 'TET'); end
        TrialData.TrialEndTime    = GetSecs;
                
        % Save Data from Single Trial
        save(...
            fullfile(DataDir,sprintf('Data%04i.mat',Trial)),...
            'TrialData',...
            '-v7.3','-nocompression');
        
        % keep track of useful stats and params
        ch_stats = TrialData.ChStats; 
        save(fullfile(Params.ProjectDir,'TaskCode','persistence','ch_stats.mat'),...
            'ch_stats','-v7.3','-nocompression');
        feature_stats = TrialData.FeatureStats;
        save(fullfile(Params.ProjectDir,'TaskCode','persistence','feature_stats.mat'),...
            'feature_stats','-v7.3','-nocompression');
        if TaskFlag>1,
            save(fullfile(Params.ProjectDir,'TaskCode','persistence','kf_params.mat'),...
                'KF','-v7.3','-nocompression');
        end
        
    end % Trial Loop
    
    % Give Feedback for Block
    if Params.InterBlockInterval >= 10,
        Instructions = [...
            sprintf('\n\nFinished block %i of %i\n\n',Block,NumBlocks),...
            '\nPress the ''Space Bar'' to resume task.' ];
        InstructionScreen(Params,Instructions)
    else,
        WaitSecs(Params.InterBlockInterval);
    end
    
end % Block Loop
%#ok<*NASGU>

end % RunLoop



