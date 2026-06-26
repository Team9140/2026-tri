package org.team9140.frc2026.subsystems.extender;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

public class Extender extends SubsystemBase{
    ExtenderIO extender;
    ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    @AutoLogOutput
    private double targetPosition;

    @AutoLogOutput
    public final Trigger atPosition = new Trigger(
            () -> Util.epsilonEquals(this.getPosition(), this.targetPosition,
                    Constants.Extender.TOLERANCE));

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

    public Command armOutForDepot() {
        return this.runOnce(() -> {
            targetPosition = Constants.Extender.DEPOT_ARM_OUT_POSITION;
            extender.goToPosition(
                Constants.Extender.DEPOT_ARM_OUT_POSITION, 
                Constants.Extender.ARM_OUT_VELOCITY);
        });
    }

    private Command armInForJiggle() {
        return this.runOnce(() -> {
            targetPosition = Constants.Extender.ARM_OUT_POSITION - Units.inchesToMeters(2);
            extender.goToPosition(
                targetPosition, 
                Constants.Extender.ARM_IN_VELOCITY);
        });
    }

    public Command jiggle() {
        return armInForJiggle()
            .andThen(new WaitUntilCommand(atPosition))
            .andThen(armOut())
            .andThen(new WaitUntilCommand(atPosition))
            .repeatedly();
    }
}
