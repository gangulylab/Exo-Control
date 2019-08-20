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

% query optimal control policy
Vopt = OptimalCursorUpdate(Params,TargetPos);

if TaskFlag==1, % do nothing during imagined movements
    Vcom = Cursor.State(2);
    Cursor.Vcommand = Vcom;
    if abs(Cursor.Vcommand)>100
        Cursor.Vcommand = 0;
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
        dx = x-Params.Center(1);
        dy = y-Params.Center(2);
        MvmtAxisUvec = [cosd(Params.MvmtAxisAngle),sind(Params.MvmtAxisAngle)];
        
        p = dot([dx,dy], MvmtAxisUvec);
        v = (p - X0(1))*Params.UpdateRate;
        
        
        % update cursor
        Cursor.State(1) = p;
        Cursor.State(2) = v;
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1) - X0(1))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        Cursor.IntendedState(2) = Vopt; % update vel w/ optimal vel
        
    case 2, % Use Mouse Position as a Velocity Input (Center-Joystick)
        X0 = Cursor.State;
        [x,y] = GetMouse();
        dx = x-Params.Center(1);
        dy = y-Params.Center(2);
        MvmtAxisUvec = [cosd(Params.MvmtAxisAngle),sind(Params.MvmtAxisAngle)];
        v = Params.Gain * dot([dx,dy],MvmtAxisUvec);
        
        % assisted velocity
        if Cursor.Assistance > 0,
            Vcom = v;
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
        else,
            Vass = v;
        end
        
        % bound at 100
        speed = abs(Vass);
        if speed>100,
            Vass = Vass * 100 / speed;
        end
        
        % update cursor state
        Cursor.State(1) = Cursor.State(1) + Vass/Params.UpdateRate;
        Cursor.State(2) = Vass;
        
        % Update Intended Cursor State
        X = Cursor.State;
        Vcom = (X(1) - X0(1))*Params.UpdateRate; % effective velocity command
        Cursor.IntendedState = Cursor.State; % current true position
        Cursor.IntendedState(2) = Vopt; % update vel w/ optimal vel
        
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
        
        % Kalman Predict Step
        X = A*X;
        P = A*P*A' + W;
        C = KF.C;
        Q = KF.Q;
        
        % Kalman Predict Step
        X = A*X;
        P = A*P*A' + W;
        P(1,:) = zeros(1,3); % zero out pos and pos-vel terms
        P(:,1) = zeros(3,1); % innovation from refit

        % Kalman Update Step
        K = P*C'/(C*P*C' + Q);
        X = X + K*(Y - C*X);
        P = P - K*C*P;
        
        % Store Params
        Cursor.State = X; % do not update the 1
        KF.P = P;
        
        % assisted velocity
        Vcom = X(2); % effective velocity command
        if Cursor.Assistance > 0,
            % Vass w/ vector avg
            %Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
            
            % Vass w/ same speed
            norm_vcom = norm(Vcom);
            Vass = Cursor.Assistance*Vopt + (1-Cursor.Assistance)*Vcom;
            Vass = norm_vcom * Vass / norm(Vass);
            
            % update cursor state
            %Cursor.State(1) = X0(1) + Vass/Params.UpdateRate;
            Cursor.State(2) = Vass;
        end
        
        % bound at 50
        speed = abs(Cursor.State(2));
        if speed>20,
            Cursor.State(2) = Cursor.State(2) * 20 / speed;
        end
        
        % Update Intended Cursor State
        Cursor.IntendedState = Cursor.State; % current true position
        Cursor.IntendedState(2) = Vopt; % update vel w/ optimal vel
        
        % Update KF Params (RML & Adaptation Block)
        if KF.CLDA.Type==3 && TaskFlag==2,
            KF = UpdateRmlKF(KF,Cursor.IntendedState,Y);
        end
        
end

% update effective velocity command for screen output
try,
    Cursor.Vcommand = Vcom;
    if abs(Cursor.Vcommand)>50
        Cursor.Vcommand = sign(Cursor.Vcommand)*50;
    end
%     fprintf('Vcom: %03.03f\n',Cursor.Vcommand);
catch,
    Cursor.Vcommand = 0;
end

% write to arduino to exo
Params.Arduino.planar.vel = [Cursor.Vcommand;0];
Params.Arduino = UpdateArduino(Params.Arduino);


% % bound cursor position to size of screen
% pos = Cursor.State(1);
% bound = min([...
%     (Params.ScreenRectangle(3)-10)-Params.Center(1),...
%     (Params.ScreenRectangle(4)-10)-Params.Center(2)]);
% pos = max([pos,-bound]); % p-left
% pos = min([pos,+bound]); % p-right
% Cursor.State(1) = pos;

% Cursor position assumes planar position

switch Params.PlanarConnected 
    case 1
        Cursor.State(1) = Params.Arduino.planar.pos(1);
end

end % UpdateCursor