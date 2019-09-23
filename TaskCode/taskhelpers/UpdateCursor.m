function [KF,Params] = UpdateCursor(Params,Neuro,TaskFlag,TargetPos,KF,Clicker)
% UpdateCursor(Params,Neuro)
% Updates the state of the cursor using the method in Params.ControlMode
%   1 - position control
%   2 - velocity control
%   3 - kalman filter  velocity
%
% Cursor - global structure with state of cursor [px,py,vx,vy,1]
% TaskFlag - 0-imagined mvmts, 1-clda, 2-fixed decoder
% TargetPos - 1d- coordinates of target position. used to assist
%   cursor to target
% KF - kalman filter struct containing matrices A,W,P,C,Q

global Cursor

% query optimal control policy; Vopt is from ReFIT
Vopt = OptimalCursorUpdate(Params,TargetPos);

if TaskFlag==1, % do nothing during imagined movements
    Vcom = [Cursor.State(3);Cursor.State(4)];
    Cursor.Vcommand = Vcom;
    if norm([Cursor.Vcommand(1),Cursor.Vcommand(2)])>100
        Cursor.Vcommand = [0;0];
    end
%     disp(Cursor.Vcommand)
    Params.Arduino.planar.vel = [Cursor.Vcommand;0];
    Params.Arduino  = UpdateArduino(Params.Arduino);
    
    return;
end

% decode click
 if Params.GenNeuralFeaturesFlag,
    [~,~,B] = GetMouse();
    if any(B),
        Clicking = -1;
    else,
        Clicking = 0;
    end
else,
    Clicking = Clicker.Func(Neuro.NeuralFeatures);
 end

% must click for X bins in a row
if Clicking==-1, % clicking
    Cursor.ClickState = Cursor.ClickState + 1;
    %Cursor.State(3:4) = 0;
    return;
else, % not clicking
    Cursor.ClickState = 0;
end


% find vx and vy using control scheme
switch Cursor.ControlMode,
    case 1, % Move to Mouse
        X0 = Cursor.State;
        [x,y] = GetMouse();
        vx = ((x-Params.Center(1)) - Cursor.State(1))*Params.UpdateRate;
        vy = ((y-Params.Center(2)) - Cursor.State(2))*Params.UpdateRate;
        %dx = x-Params.Center(1);
        %dy = y-Params.Center(2);
        %MvmtAxisUvec = [cosd(Params.MvmtAxisAngle),sind(Params.MvmtAxisAngle)];
        
        %p = dot([dx,dy], MvmtAxisUvec);
        %v = (p - X0(1))*Params.UpdateRate;
        
        % update cursor
        Cursor.State(1) = x - Params.Center(1);
        Cursor.State(2) = y - Params.Center(2);
        Cursor.State(3) = vx;
        Cursor.State(4) = vy;
        
        % update cursor
        %Cursor.State(1) = p;
        %Cursor.State(2) = v;
        
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1:2) - X0(1:2))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        Cursor.IntendedState(3:4) = Vopt; % update vel w/ optimal vel
        
    case 2, % Use Mouse Position as a Velocity Input (Center-Joystick)
        X0 = Cursor.State;
        [x,y] = GetMouse();
        vx = Params.Gain * (x - Params.Center(1));
        vy = Params.Gain * (y - Params.Center(2));
        
        % assisted velocity
        if Cursor.Assistance > 0,
            Vcom = [vx;vy];
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
        else,
            Vass = [vx;vy];
        end
        
        % bound at 100
        speed = norm([Vass(1),Vass(2)]);
        if speed>50,
            Vass = Vass * 50 / speed;
        end
        
        % update cursor state
        Cursor.State(1) = Cursor.State(1) + Vass(1)/Params.UpdateRate;
        Cursor.State(2) = Cursor.State(2) + Vass(2)/Params.UpdateRate;
        Cursor.State(3) = Vass(1);
        Cursor.State(4) = Vass(2);
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1:2) - X0(1:2))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        Cursor.IntendedState(3:4) = Vopt; % update vel w/ optimal vel

    case {3,4}, % Kalman Filter Input
        X0 = Cursor.State; % initial state, useful for assistance
        
        X = X0;
        if Neuro.DimRed.Flag,
            Y = Neuro.NeuralFactors;
        else,
            Y = Neuro.NeuralFeatures;
        end
        A = KF.A;
        W = KF.W;
        P = KF.P;
        C = KF.C;
        Q = KF.Q;
        
        % Kalman Predict Step
        X = A*X;
        P = A*P*A' + W;
        P(1:2,:) = zeros(2,5); % zero out pos and pos-vel terms
        P(:,1:2) = zeros(5,2); % innovation from refit
        
        % Kalman Update Step
        K = P*C'/(C*P*C' + Q);
        X = X + K*(Y - C*X);
        P = P - K*C*P;
        
        % Store Params
        Cursor.State = X; % do not update the 1
        KF.P = P;
        KF.K = K;
        
        % assisted velocity
        % Vcom is from kalman Filter
        Vcom = X(3:4); % effective velocity command
        if Cursor.Assistance > 0,
            % Vass w/ vector avg
            %Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
            
            % Vass w/ same speed
            % Vcom is from kalman Filter and Vopt is from ReFIT
            norm_vcom = norm(Vcom);
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
            Vass = norm_vcom * Vass / norm(Vass);
            
            % update cursor state
            %Cursor.State(1) = X0(1) + Vass/Params.UpdateRate;
            Cursor.State(3) = Vass(1);
            Cursor.State(4) = Vass(2);
        end
        
        % bound at 50
        speed = norm([Cursor.State(3),Cursor.State(4)]);
        if speed>20,
            Cursor.State(3) = Cursor.State(3) * 20 / speed;
            Cursor.State(4) = Cursor.State(4) * 20 / speed;
        end
        
        % Update Intended Cursor State
        Cursor.IntendedState = Cursor.State; % for position; current true position
        Cursor.IntendedState(3:4) = Vopt; % update vel w/ optimal vel
        
        % Update KF Params (RML & Adaptation Block)
        if KF.CLDA.Type==3 && TaskFlag==2,
            KF =  UpdateRmlKF(KF,Cursor.IntendedState,Y,Params,TaskFlag);
        end
        
end

% update effective velocity command for screen output; 
% This is (Vcom, the velocity from Kalman Filter to update the arrow indicator in the corner of screen
% The real output position parameters (Cursor.State(1),Cursor.State(2)) is used for cursor
% update on screen
% The real Cursor Velocity (included V assist!) [Cursor.State(3:4)] is used
% to send to exo 

try,
    Cursor.Vcommand = Cursor.State(3:4);
    if norm([Cursor.Vcommand(1),Cursor.Vcommand(2)])>50
        Cursor.Vcommand = [sign(Cursor.Vcommand(1))*50, sign(Cursor.Vcommand(2))*50];
    end
%     fprintf('Vcom: %03.03f\n',Cursor.Vcommand);
catch,
    Cursor.Vcommand = [0;0];
end

% write to arduino to exo
Params.Arduino.planar.vel = [Cursor.Vcommand(1);-Cursor.Vcommand(2)];
Params.Arduino = UpdateArduino(Params.Arduino);

% Cursor position is updated using feedback from planar position

% updating the cursor position value using planar exo's position.
switch Params.PlanarConnected 
    case 1
        Cursor.State(1) = Params.Arduino.planar.pos(1);
end

% bound cursor position to size of screen/monitor
if 0
    pos = Cursor.State(1:2)' + Params.Center;
    pos(1) = max([pos(1),Params.ScreenRectangle(1)+10]); % x-left
    pos(1) = min([pos(1),Params.ScreenRectangle(3)-10]); % x-right
    pos(2) = max([pos(2),Params.ScreenRectangle(2)+10]); % y-left
    pos(2) = min([pos(2),Params.ScreenRectangle(4)-10]); % y-right
    Cursor.State(1) = pos(1) - Params.Center(1);
    Cursor.State(2) = pos(2) - Params.Center(2);
end


% bound cursor position to size of planar Exo
pos = Cursor.State(1:2)' + Params.Center;
pos(1) = max([pos(1),Params.Center(1)-300]); % x-left
pos(1) = min([pos(1),Params.Center(1)+300]); % x-right
pos(2) = max([pos(2),Params.Center(2)-150]); % y-left
pos(2) = min([pos(2),Params.Center(2)+150]); % y-right
Cursor.State(1) = pos(1) - Params.Center(1);
Cursor.State(2) = pos(2) - Params.Center(2);






end % UpdateCursor