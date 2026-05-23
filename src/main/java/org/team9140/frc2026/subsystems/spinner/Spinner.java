package org.team9140.frc2026.subsystems.spinner;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase{
    private SpinnerIO spinnerMotor;
    private SpinnerIOInputsAutoLogged inputs = new SpinnerIOInputsAutoLogged();

    public Spinner(SpinnerIO io) {
        this.spinnerMotor = io;
    }

    @Override
    public void periodic() {
        spinnerMotor.updateInputs(inputs);
        Logger.processInputs("Spinner", inputs);
    }

    public Command feed() {
        return this.runOnce(() -> {
            spinnerMotor.runVoltage(Constants.Spinner.VOLTAGE);
        });
    }

    public Command unjam() {
        return this.runOnce(() -> {
            spinnerMotor.runVoltage(Constants.Spinner.REVERSE_VOLTAGE);
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            spinnerMotor.off();
        });
    }
}
