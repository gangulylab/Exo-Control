function [Data, Neuro, KF, Params,Clicker] = RunTrial(Data,Params,Neuro,TaskFlag,KF,Clicker)
% Runs a trial, saves useful data along the way
% Each trial contains the following pieces
% 1) Inter-trial interval
% 2) Get the cursor to the star t target (center)
% 3) Hold position during an instructed delay period
% 4) Get the cursor to the reach target (different on each trial)
% 5) Feedback

global Cursor

%% Set up trial
StartTargetPos = Params.StartTargetPosition;
ReachTargetPos = Data.TargetPosition;

% Output to Command Line
fprintf('\nTrial: %i\n',Data.Trial)
fprintf('  Target: %i\n',Data.TargetPosition)
if Params.Verbose,
    if TaskFlag==2,
        fprintf('    Cursor Assistance: %i%%\n',round(100*Cursor.Assistance))
        if Params.CLDA.Type==3,
            %fprintf('    Lambda 1/2 life: %.2fsecs\n',log(.5)/log(KF.Lambda)/Params.UpdateRate)
            %fprintf('    Lambda 1/2 life: %.2fsecs\n',KF.Lambda)
        end
    end
end
 
% keep track of update times
dt_vec = [];
dT_vec = [];

% grab blackrock data and run through processing pipeline
if Params.BLACKROCK,
    Cursor.LastPredictTime = GetSecs;
    Cursor.LastUpdateTime = Cursor.LastPredictTime;
    Neuro = NeuroPipeline(Neuro);
end

%%==========================================================================================================================
%% TRIAL CODE START
%%==========================================================================================================================

protocolSelect = 4;
switch protocolSelect
    case 0
        s_protocol1DClose;
    case 1
        s_protocolRGRR;
    case 2
        fprintf('\n\tProtocol: Volitional Reach & Return\n')
        s_protocolRR;
    case 3
        fprintf('\n\tProtocol: Volitional Reach, Automated Grasp, Volitional Return & Automatic Release\n')
        s_protocolVRAGVRAR;
        
    case 4
        fprintf('\n\tProtocol: Volitional Reach, Click Grasp, Volitional Return & Click Release\n')
        s_protocolVRAGVRARClick;
end


%%==========================================================================================================================
%% TRIAL CODE END
%%================================================================================================================

%% Completed Trial - Give Feedback
Screen('Flip', Params.WPTR);

% output update times
if Params.Verbose,
    fprintf('      Screen Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.ScreenRefreshRate,mean(1./dt_vec),std(1./dt_vec))
    fprintf('      System Update: Goal=%iHz, Actual=%.2fHz (+/-%.2fHz)\n',...
        Params.UpdateRate,mean(1./dT_vec),std(1./dT_vec))
end

% output feedback
if Data.ErrorID==0,
    fprintf('SUCCESS\n')
    if Params.FeedbackSound,
        sound(Params.RewardSound,Params.RewardSoundFs)
    end
else
    % reset cursor
    Cursor.ClickState = 0;
    
    fprintf('ERROR: %s\n', Data.ErrorStr)

    if Params.FeedbackSound,
        sound(Params.ErrorSound,Params.ErrorSoundFs)
    end
    WaitSecs(Params.ErrorWaitTime);
end

end % RunTrial
