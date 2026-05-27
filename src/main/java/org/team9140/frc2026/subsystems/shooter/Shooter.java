package org.team9140.frc2026.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.helpers.LookUpAimAlign;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase{
    private ShooterIO shooterMotor;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @AutoLogOutput
    private double targetVelocity = 0.0;
    LinearFilter RPMFilter = LinearFilter.movingAverage(20);
    @AutoLogOutput
    private final Trigger atVelocity = new Trigger(() -> {
        return RPMFilter.calculate(inputs.velocityRotsPerSec) >= targetVelocity - 4.0;
    });

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

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;

    public void setRobotDataSuppliers(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds) {
        this.robotPoseSupplier = robotPose;
        this.robotSpeedsSupplier = robotSpeeds;
    }

    public Command aim(Supplier<Translation2d> targetTranslation) {
        return this.run(() -> {
            Pose2d turretPose = this.robotPoseSupplier.get();

            Translation2d targetPose = targetTranslation.get();
            boolean isPassing = !targetPose.equals(LookUpAimAlign.getHub().getTranslation());

            targetPose = LookUpAimAlign.getEffectivePose(turretPose,
                    targetPose, this.robotSpeedsSupplier.get(), isPassing);
            
            targetVelocity = LookUpAimAlign.getRequiredSpeed(turretPose, targetPose, isPassing);
            shooterMotor.goToVelocity(targetVelocity);
            
        });
    }

    public Command aim() {
        return this.aim(() -> LookUpAimAlign.getZone(robotPoseSupplier.get()).getTranslation());
    }
}
