package org.team9140.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private HoodIO hoodMotor;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    @AutoLogOutput
    private double targetPos;

    public Hood(HoodIO io) {
        this.hoodMotor = io;
    }

    @Override
    public void periodic() {
        hoodMotor.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public void setHoodPosition(double angleDegrees) {
        double pos = angleDegrees / 360.0 - Units.radiansToRotations(Constants.Hood.ANGLE_MIN);
        this.targetPos = pos;
        this.hoodMotor.moveToPosition(pos);
        
    }

    public Command off() {
        return this.runOnce(() -> {
            setHoodPosition(30);
        });
    }
}
