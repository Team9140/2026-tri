package org.team9140.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Feeder extends SubsystemBase{
    private FeederIO feederMotor;
    private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public Feeder(FeederIO io) {
        this.feederMotor = io;
    }

    @Override
    public void periodic() {
        feederMotor.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public Command feed() {
        return this.runOnce(() -> {
            feederMotor.runVoltage(Constants.Feeder.VOLTAGE);
        });
    }

    public Command unjam() {
        return this.runOnce(() -> {
            feederMotor.runVoltage(-Constants.Feeder.VOLTAGE);
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            feederMotor.off();
        });
    }

    public Command reverseAndOff() {
        return unjam()
            .andThen(new WaitCommand(Constants.Feeder.REVERSE_FEEDER_TIME))
            .andThen(off());
    }
}
