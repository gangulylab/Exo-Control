% s_protocolVRAGVRAR
% Volitional reach, auto-grasp, volitional return, auto release

% Move Planar system to the home position then disable it.
Params.Arduino.planar.target        = 0;    % 1-bit     Move to Home
Params.Arduino.glove.enable         = 1;    % 1-bit     Enable Glove
Params.Arduino.glove.admittanceMode = 1;    % 1-bit     Set glove to admittance mode to reduce strain in hand

% initialization; reset the planar exo to the home position 
s_planarForceState;

% Offer tempoary respite between trials
s_interTrialInterval;

% Instruct to go to reach target
idx = Data.TargetID;
switch idx
    case 0
        Params.Arduino.planar.target           = 0;
    case 1 % Left Target
        Params.Arduino.planar.target           = 5; % Go West
    case 2 % Right Target
        Params.Arduino.planar.target           = 1; % Go East
    otherwise
        Params.Arduino.planar.target           = 0;
end

% initialization; Changing the LEDs 
s_planarInstructTarget;
Params.successfulReach = 0;

% Perform the following tasks in order

% Set glove to open
if Data.ErrorID==0,
    Params.Arduino.glove.target = 2;
    s_gloveForceState
end

% Allow volitional movements
s_planarVolitional;

% Force to target
if Params.successfulReach == 0
    s_planarForceState;
end

% Set glove to close
<<<<<<< Updated upstream
if Data.ErrorID==0,
    Params.Arduino.glove.target = 0;
    s_gloveForceState;
end
=======
if Params.successfulReach == 1
    Params.Arduino.glove.target = 0;
    s_gloveForceState;
end

>>>>>>> Stashed changes

% Instruct to go Home
Params.Arduino.planar.target = 0;
s_planarInstructTarget;
Params.successfulReach = 0;

% Allow volitional movements
s_planarVolitional;

% Set glove to open
if Data.ErrorID==0,
    Params.Arduino.glove.target = 2;
    s_gloveForceState
end

% On completion of attempted return motion, disable planar and switch to position mode
Params.Arduino.planar.enable        = 0;    % 1-bit     Disable planar
Params.Arduino.planar.velocityMode  = 0;    % 1-bit     Set planar to position mode
Params.Arduino.glove.enable         = 1;    % 1-bit     Enable Glove
Params.Arduino.glove.admittanceMode = 1;    % 1-bit     Set glove to admittance mode to reduce strain in hand
Params.Arduino = UpdateArduino(Params.Arduino);

