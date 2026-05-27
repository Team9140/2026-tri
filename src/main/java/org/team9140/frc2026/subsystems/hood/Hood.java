package org.team9140.frc2026.subsystems.hood;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;
import org.team9140.frc2026.helpers.LookUpAimAlign;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hood extends SubsystemBase{
    private HoodIO hoodMotor;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    @AutoLogOutput
    private double targetAngleRotations;
    @AutoLogOutput
    public Trigger atPosition = new Trigger(() -> {
        return Util.epsilonEquals(inputs.hoodAngleRotations, targetAngleRotations, Constants.Hood.TOLERANCE_ROTATIONS);
    });

    public Hood(HoodIO io) {
        this.hoodMotor = io;
    }

    @Override
    public void periodic() {
        hoodMotor.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    private void moveToPosition(double angleDegrees) {
        double pos = angleDegrees / 360.0 - Units.radiansToRotations(Constants.Hood.ANGLE_MIN);
        this.targetAngleRotations = pos;
        this.hoodMotor.moveToPosition(pos);
        
    }

    public Command tuning(DoubleSupplier angleDegrees) {
        return this.runOnce(() -> {
            moveToPosition(angleDegrees.getAsDouble());
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            moveToPosition(30);
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
            
            this.moveToPosition(LookUpAimAlign.getRequiredHoodAngle(turretPose, targetPose, isPassing));
        });
    }

    public Command aim() {
        return aim(() -> LookUpAimAlign.getZone(robotPoseSupplier.get()).getTranslation());
    }
}
