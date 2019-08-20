clear all
clc
pause(0.001)
clear all


%%
Params.ArduinoPtr = arduino('COM41','Due','Libraries','I2C');
Params.ArduinoPin = 'D13';
writeDigitalPin(Params.ArduinoPtr, Params.ArduinoPin, 0); % make sure the pin is at 0
PulseArduino(Params.ArduinoPtr,Params.ArduinoPin,20);
Params.Arduino.devBBS           = i2cdev(Params.ArduinoPtr,'0x01','bus',0); % BBS is on Brain Box bus I2C0, device 1


%%
Params.Arduino.planar.velParams.minSpeed     = -100; % mm/s
Params.Arduino.planar.velParams.maxSpeed     =  100; % mm/s
Params.Arduino.planar.velParams.bits         =  12; % bits
Params.Arduino.planar.velParams.f_speed2bits = @(speed) round(((speed-Params.Arduino.planar.velParams.minSpeed)...
                                                ./(Params.Arduino.planar.velParams.maxSpeed-Params.Arduino.planar.velParams.minSpeed))...
                                                .*(2^Params.Arduino.planar.velParams.bits-1));

Params.Arduino.planar.posParams.minPos       = -300; % mm
Params.Arduino.planar.posParams.maxPos       =  300; % mm
Params.Arduino.planar.posParams.bits         =  12; % bits
Params.Arduino.planar.posParams.f_bits2pos   = @(bits) ((double(bits)./(2^Params.Arduino.planar.posParams.bits-1))...
                                                .*(Params.Arduino.planar.posParams.maxPos-Params.Arduino.planar.posParams.minPos)...
                                                +Params.Arduino.planar.posParams.minPos);

%%
Params.Arduino.glove.target         =  2;    % 4-bits    For 16 targets
                                            % 0: Close, 2: Open 
    
Params.Arduino.planar.enable        = 1;    % 1-bit     Turn planar on and off
Params.Arduino.planar.velocityMode  = 0;    % 1-bit     Run in Velocity or Target mode
Params.Arduino.planar.target        = 0;    % 4-bits    For 16 targets

Params.Arduino.planar.vel           = [0;0]; % mm/s 12-bits    0 - 4095

Params.Arduino.glove.enable         = 1;    % 1-bit     Turn glove on and off
Params.Arduino.glove.admittanceMode = 0;    % 1-bit     Run admittance or target mode
% if Params.Arduino.glove.target == 0;    % 4-bits    For 16 targets
%     Params.Arduino.glove.target = 2;    % 4-bits    For 16 targets 
% else
%     Params.Arduino.glove.target = 0;    % 4-bits    For 16 targets
% end

fprintf('============================================\n')
fprintf('Command Vel: (%03.03f,%03.03f) mm/s \n',Params.Arduino.planar.vel(1),Params.Arduino.planar.vel(2))
[Params.Arduino] = UpdateArduino(Params.Arduino);
% Params.Arduino  = Arduino;

fprintf('Planar Pos: (%03.03f,%03.03f) mm \n',Params.Arduino.planar.pos(1),Params.Arduino.planar.pos(2))
fprintf('Glove Pos: %03.03f, Glove Force: %03.03f  \n',Params.Arduino.glove.pos,Params.Arduino.glove.force)

% pause(5)