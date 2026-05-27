package org.team9140.frc2026.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.LookUpAimAlign;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Turret extends SubsystemBase {
    TurretIO turretMotor;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    @AutoLogOutput
    private double targetPosition;
    @AutoLogOutput
    public final Trigger atPosition = new Trigger(
            () -> Util.epsilonEquals(this.getPosition(),
                    targetPosition, Units.degreesToRotations(15.0)));

    public Turret(TurretIO io) {
        this.turretMotor = io;
    }

    @Override
    public void periodic() {
        turretMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Pose2d turretPos = robotPoseSupplier.get().plus(Constants.Turret.POSITION_TO_ROBOT);
        turretPos = turretPos.plus(new Transform2d(new Translation2d(),
                            new Rotation2d(this.getPosition() * 2 * Math.PI)));
        Logger.recordOutput("Turret Position Visualized", turretPos);

    }

    public double getPosition() {
        return inputs.turretAngleRotations;
    }

    public Command manualAdjust(boolean left) {
        return this.runOnce(() -> {
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
            turretMotor.moveToPosition(0);
            this.targetPosition = 0.0;
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
            
            double yaw = LookUpAimAlign.yawAngleToPos(turretPose, targetPose) / (2.0 * Math.PI);

            if (getPosition() > 0 && yaw < -160.0 / 360.0) {
                yaw += 1;
            } else if (getPosition() < 0 && yaw > 160.0 / 360.0) {
                yaw -= 1;
            }
            targetPosition = yaw;
            turretMotor.moveToPosition(targetPosition);
        });
    }

    public Command aim() {
        return aim(() -> LookUpAimAlign.getZone(robotPoseSupplier.get()).getTranslation());
    }

    
    
}
