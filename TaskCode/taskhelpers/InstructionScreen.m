function InstructionScreen(Params,tex)
% Display text then wait for subject to resume experiment

Params.Arduino.planar.enable = 0;
Params.Arduino  = UpdateArduino(Params.Arduino);

% Pause Screen
DrawFormattedText(Params.WPTR, tex,'center','center',255);
Screen('Flip', Params.WPTR);

WaitSecs(.1);

while (1) % pause until subject presses spacebar to continue
    [~, ~, keyCode, ~] = KbCheck;
    if keyCode(KbName('space'))==1,
        keyCode(KbName('space'))=0;
        break;
    elseif keyCode(KbName('escape'))==1,
        ExperimentStop(1,Params);
    end
end

Screen('Flip', Params.WPTR);
WaitSecs(.1);

end % InstructionScreen