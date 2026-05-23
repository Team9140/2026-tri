package org.team9140.frc2026.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    TurretIO turretMotor;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    @AutoLogOutput
    boolean isManual = false;
    @AutoLogOutput
    double targetPosition;

    public Turret(TurretIO io) {
        this.turretMotor = io;
    }

    @Override
    public void periodic() {
        turretMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public double getPosition() {
        return inputs.turretAngleRotations;
    }

    public Command manualAdjust(boolean left) {
        return this.runOnce(() -> {
            this.isManual = true;
            this.targetPosition = 0.0;
            turretMotor.runVoltage(
                left ? Constants.Turret.ADJUST_VOLTAGE : -Constants.Turret.ADJUST_VOLTAGE);
        });
    }

    public Command adjustLeft() {
        return manualAdjust(true);
    }

    public Command adjustRight() {
        return manualAdjust(false);
    }

    public Command brake() {
        return this.runOnce(() -> {
            turretMotor.brake();
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            if (this.isManual) return;
            turretMotor.moveToPosition(0);
            this.targetPosition = 0.0;
        });
    }
/* For testing purposes
    public Command goTo180() {
        return this.runOnce(() -> {
            if (this.isManual) return;
            turretMotor.moveToPosition(0.5);
            this.targetPosition = 0.5;
        });
    }
*/
    
    
}
