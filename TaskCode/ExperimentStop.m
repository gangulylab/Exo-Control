function ExperimentStop(fromPause,Params)
if ~exist('fromPause', 'var'), fromPause = 0; end

% Put the planar into a safe state
Params.Arduino = UpdateArduino(Params.Arduino);
while Params.Arduino.planar.ready == 0
    Params.Arduino.planar.enable        = 1;    % Enable Planar
    Params.Arduino.planar.velocityMode  = 0;    % Set to position mode
    Params.Arduino.planar.target        = 0;    % Set target as home
    Params.Arduino = UpdateArduino(Params.Arduino);
    Screen('DrawText', Params.WPTR, 'Waiting for planar to disable...',50, 50, [255,0,0], [0,0,0]);
    Params.Arduino = UpdateArduino(Params.Arduino);
    Screen('Flip', Params.WPTR);
    pause(0.1)
end
Params.Arduino.planar.enable        = 0;    % Disable Planar
Params.Arduino.glove.enable         = 0;
Params.Arduino = UpdateArduino(Params.Arduino);

% Close Screen
Screen('CloseAll');

% Close Serial Port
fclose('all');

% quit
fprintf('Ending Experiment\n')
if fromPause, keyboard; end

end % ExperimentStop
