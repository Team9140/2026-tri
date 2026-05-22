package org.team9140.frc2026.subsystems.extender;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

public class Extender extends SubsystemBase{
    ExtenderIO extender;
    ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    public Extender(ExtenderIO extender) {
        this.extender = extender;
    }

    @Override
    public void periodic() {
        extender.updateInputs(inputs);
        Logger.processInputs("Extender", inputs);
    }

    public Command armIn() {
        return this.runOnce(() -> {
            extender.setPosition(
                Constants.Extender.ARM_IN_POSITION, 
                Constants.Extender.ARM_IN_VELOCITY);
        });
    }

    public Command armOut() {
        return this.runOnce(() -> {
            extender.setPosition(
                Constants.Extender.ARM_OUT_POSITION, 
                Constants.Extender.ARM_OUT_VELOCITY);
        });
    }

    public Command armInOutLoop() {
        return this.run(() -> {
            armIn().withTimeout(1).andThen(armOut().withTimeout(1)).repeatedly();
        });
    }
}
