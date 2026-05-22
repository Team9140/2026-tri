package org.team9140.frc2026.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

public class Roller extends SubsystemBase{
    RollerIO roller;
    RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public Roller(RollerIO roller) {
        this.roller = roller;
    }

    @Override
    public void periodic() {
        roller.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);
    }

    public Command stop() {
        return this.runOnce(() -> {
            roller.off();
        });
    }

    public Command intake() {
        return this.runOnce(() -> {
            roller.runVoltage(Constants.Roller.INTAKING_ROLLER_VOLTAGE);
        });
    }

    public Command reverse() {
        return this.runOnce(() -> {
            roller.runVoltage(-Constants.Roller.INTAKING_ROLLER_VOLTAGE);
        });
    }
}
