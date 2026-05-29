package org.team9140.frc2026.subsystems.drive;

import org.team9140.frc2026.generated.TunerConstants.TunerSwerveDrivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SwerveDriveIOReal extends TunerSwerveDrivetrain implements SwerveDriveIO{

    public SwerveDriveIOReal(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
    }

    @Override
    public void updateInputs(SwerveDriveIOInputs inputs) {
        SwerveDriveState driveState = this.getState();
        inputs.Pose = driveState.Pose;
        inputs.Speeds = driveState.Speeds;
        inputs.ModuleStates = driveState.ModuleStates;
        inputs.ModuleTargets = driveState.ModuleTargets;
        inputs.ModulePositions = driveState.ModulePositions;
        inputs.Timestamp = driveState.Timestamp;
        inputs.OdometryPeriod = driveState.OdometryPeriod;
        inputs.SuccessfulDaqs = driveState.SuccessfulDaqs;
        inputs.FailedDaqs = driveState.FailedDaqs;
        inputs.RawHeading = driveState.RawHeading;
    }
    
    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double fpgaTimestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, 
            Utils.fpgaToCurrentTime(fpgaTimestampSeconds), 
            visionMeasurementStdDevs);
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }
}
