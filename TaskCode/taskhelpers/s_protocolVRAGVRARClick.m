% s_protocolVRAGVRARClick
% Volitional reach, auto-grasp, volitional return, auto release

% Move Planar system to the home position then disable it.
Params.Arduino.planar.target        = 0;    % 1-bit     Move to Home postion
Params.Arduino.glove.enable         = 1;    % 1-bit     Enable Glove
Params.Arduino.glove.admittanceMode = 1;    % 1-bit     Set glove to admittance mode to reduce strain in hand

s_planarForceState;

s_touchCursor;

% Offer tempoary respite between trials
s_interTrialInterval;
s_touchCursor;

% Instruct to go to reach target or return to the center
idx = Data.TargetID; % idx=0 means center target; otherwise center-out targets
disp(idx)

switch idx
    case 0 % for returning to the center target (home position)
        Params.Arduino.planar.target           = 0;

    otherwise % for moving towards target positions around the circle
        Params.Arduino.planar.target           = 1;
end

s_planarInstructTarget;
Params.successfulReach = 0;
s_touchCursor;

% Set glove to open
Params.Arduino.glove.target = 2;
s_gloveForceState
s_touchCursor;

% Allow volitional movements
s_planarVolitionalClick;
s_touchCursor;

% Force to target
if Params.successfulReach == 0
    s_planarForceState;
    s_touchCursor;
end

% Set glove to close
if Params.successfulReach == 1
    Params.Arduino.glove.target = 0;
    s_gloveForceState;
    s_touchCursor;
end


% Instruct to go Home position (center target)
Params.Arduino.planar.target = 0;
s_planarInstructTarget;
Params.successfulReach = 0;
s_touchCursor;

% Allow volitional movements
s_planarVolitionalClick;
s_touchCursor;


% On completion of attempted return motion, disable planar and switch to position mode
Params.Arduino.planar.enable        = 0;    % 1-bit     Disable planar
Params.Arduino.planar.velocityMode  = 0;    % 1-bit     Set planar to position mode
Params.Arduino.glove.enable         = 1;    % 1-bit     Enable Glove
Params.Arduino.glove.admittanceMode = 1;    % 1-bit     Set glove to admittance mode to reduce strain in hand
Params.Arduino = UpdateArduino(Params.Arduino);
s_touchCursor;

