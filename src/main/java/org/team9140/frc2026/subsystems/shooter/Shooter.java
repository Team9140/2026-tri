package org.team9140.frc2026.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private ShooterIO shooterMotor;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @AutoLogOutput
    private double targetVelocity = 0.0;

    public Shooter(ShooterIO io) {
        this.shooterMotor = io;
    }

    @Override
    public void periodic() {
        shooterMotor.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command off() {
        return this.runOnce(() -> {
            shooterMotor.off();
        });
    }

    public Command tuning(DoubleSupplier RPM) {
        return this.runOnce(() -> {
            targetVelocity = RPM.getAsDouble() / 60;
            shooterMotor.goToVelocity(targetVelocity);
        });
    }
}
