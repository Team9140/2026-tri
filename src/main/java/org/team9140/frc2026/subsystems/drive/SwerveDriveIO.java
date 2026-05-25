package org.team9140.frc2026.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveDriveIO {

    @AutoLog
    static class SwerveDriveIOInputs {
        // drivetrain data
        Pose2d drivePose = new Pose2d();
        ChassisSpeeds driveSpeeds = new ChassisSpeeds();
        SwerveModuleState[] driveModuleStates;
        SwerveModuleState[] driveModuleTargets;
        SwerveModulePosition[] driveModulePositions;
        double driveTimestamp;
        double driveOdometryFrequency;

        // We never logged pigeon data in the old code so idk what we want

    }

    default void updateInputs(SwerveDriveIOInputs inputs) {}

    default void setControl(SwerveRequest request) {}

    default void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double fpgaTimestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {}
} 
