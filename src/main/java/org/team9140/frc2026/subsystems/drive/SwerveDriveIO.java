package org.team9140.frc2026.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveDriveIO {

    @AutoLog
    static class SwerveDriveIOInputs extends SwerveDriveState{}

    default void updateInputs(SwerveDriveIOInputs inputs) {}

    default void setControl(SwerveRequest request) {}

    default void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double fpgaTimestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {}
} 
