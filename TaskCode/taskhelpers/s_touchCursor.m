switch Params.PlanarConnected
    case 1
        [Params.Arduino] = UpdateArduino(Params.Arduino);
        Cursor.State(1) = Params.Arduino.planar.pos(1);
        Cursor.State(2) = -Params.Arduino.planar.pos(2)+Params.Arduino.planar.yOffset;
end