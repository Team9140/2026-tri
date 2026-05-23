package org.team9140.frc2026.subsystems.extender;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

public class Extender extends SubsystemBase{
    ExtenderIO extender;
    ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    @AutoLogOutput
    private double targetPosition;

    public Extender(ExtenderIO extender) {
        this.extender = extender;
    }

    public double getPosition() {
        return inputs.intakePosition;
    }

    @Override
    public void periodic() {
        extender.updateInputs(inputs);
        Logger.processInputs("Extender", inputs);
    }

    public Command armIn() {
        return this.runOnce(() -> {
            targetPosition = Constants.Extender.ARM_IN_POSITION;
            extender.goToPosition(
                Constants.Extender.ARM_IN_POSITION, 
                Constants.Extender.ARM_IN_VELOCITY);
        });
    }

    public Command armOut() {
        return this.runOnce(() -> {
            targetPosition = Constants.Extender.ARM_OUT_POSITION;
            extender.goToPosition(
                Constants.Extender.ARM_OUT_POSITION, 
                Constants.Extender.ARM_OUT_VELOCITY);
        });
    }

    public Command armInOutLoop() {
        return armIn()
            .andThen(new WaitCommand(1))
            .andThen(armOut())
            .andThen(new WaitCommand(1))
            .repeatedly();
    }
}
