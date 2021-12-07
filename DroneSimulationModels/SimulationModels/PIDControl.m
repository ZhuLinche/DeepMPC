function PIDControlValue = PIDControl(Kp, Ki, Kd, PositionError, IntError, Velocity)
    PIDControlValue = Kp*PositionError+Ki*IntError-Kd*Velocity;
end