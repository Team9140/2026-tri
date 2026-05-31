package org.team9140.frc2026.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants.Drive;
import org.team9140.lib.Util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CommandSwerveDrivetrain extends SubsystemBase{
    private SwerveDriveIO drivetrain;
    private SwerveDriveIOInputsAutoLogged inputs = new SwerveDriveIOInputsAutoLogged();

    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.FieldCentric auton;
    private final SwerveDriveBrake xBrake;

    private final PhoenixPIDController autonXController;
    private final PhoenixPIDController autonYController;
    private final PhoenixPIDController autonHeadingController;

    @AutoLogOutput
    private Pose2d autonTargetPose = new Pose2d();
    @AutoLogOutput
    public final Trigger reachedAutonPose;
    @AutoLogOutput
    private double driveAutonXerror;
    @AutoLogOutput
    private double driveAutonYerror;
    @AutoLogOutput
    private double driveAutonThetaError;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization;
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization;
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization;

    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineTranslation;

    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineSteer;

    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineRotation;

    // the one we actually are testing
    private SysIdRoutine sysIdRoutineToApply;

    public CommandSwerveDrivetrain(SwerveDriveIO io) {
        this.drivetrain = io;

        drive = new SwerveRequest.FieldCentric()
            .withDeadband(Drive.MIN_TELEOP_VELOCITY)
            .withRotationalDeadband(Drive.MIN_TELEOP_ROTATION)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

        auton = new SwerveRequest.FieldCentric()
            .withDeadband(Drive.MIN_AUTON_VELOCITY)
            .withRotationalDeadband(Drive.MIN_AUTON_ROTATION)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);
        
        xBrake = new SwerveDriveBrake();
        
        autonXController = new PhoenixPIDController(Drive.X_CONTROLLER_P,
            Drive.X_CONTROLLER_I, Drive.X_CONTROLLER_D);
        autonYController = new PhoenixPIDController(Drive.Y_CONTROLLER_P,
            Drive.Y_CONTROLLER_I, Drive.Y_CONTROLLER_D);
        autonHeadingController = new PhoenixPIDController(Drive.HEADING_CONTROLLER_P,
            Drive.HEADING_CONTROLLER_I, Drive.HEADING_CONTROLLER_D);
        autonHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        reachedAutonPose = new Trigger(() -> Util.epsilonEquals(autonTargetPose, inputs.Pose))
            .debounce(0.25, DebounceType.kBoth);
        
        /* Swerve requests to apply during SysId characterization */
        translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
        steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
        rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

        /*
         * SysId routine for characterizing translation. This is used to find PID gains
         * for the drive motors.
         */
        sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> Logger.recordOutput("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> drivetrain.setControl(translationCharacterization.withVolts(output)),
                    null,
                    this));
        
        /*
         * SysId routine for characterizing steer. This is used to find PID gains for
         * the steer motors.
         */ 
        sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(3).per(Second), // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> Logger.recordOutput("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> drivetrain.setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this));
        
        /*
         * SysId routine for characterizing rotation.
         * This is used to find PID gains for the FieldCentricFacingAngle
         * HeadingController.
         * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
         * importing the log to SysId.
         */
        sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> Logger.recordOutput("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        drivetrain.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        Logger.recordOutput("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));
        
        sysIdRoutineToApply = sysIdRoutineTranslation;
    }

    @Override
    public void periodic() {
        drivetrain.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);

        driveAutonXerror = autonTargetPose.getX() - inputs.Pose.getX();
        driveAutonYerror = autonTargetPose.getY() - inputs.Pose.getY();
        driveAutonThetaError = autonTargetPose.getRotation().getRotations() - inputs.Pose.getRotation().getRotations();
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    // We can add more getters if needed
    public Pose2d getDrivetrainPose() {
        return inputs.Pose;
    }

    public ChassisSpeeds getDrivetrainSpeeds() {
        return inputs.Speeds;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    private double startTime = 0.0;

    private DoubleSupplier leftStickX, leftStickY, rightStickX;

    public void setJoystickInput(DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
    }

    private Command drive(double teleopVelocityMultiplier) {
        return this.runOnce(() -> {
            startTime = Utils.getSystemTimeSeconds();
        }).andThen(this.run(() -> {
            double vX = teleopVelocityMultiplier * Drive.MAX_TELEOP_VELOCITY
                    * MathUtil.applyDeadband(-this.leftStickY.getAsDouble(), Drive.CONTROLLER_DEADBAND);
            double vY = teleopVelocityMultiplier * Drive.MAX_TELEOP_VELOCITY
                    * MathUtil.applyDeadband(-this.leftStickX.getAsDouble(), Drive.CONTROLLER_DEADBAND);
            double omega = Drive.MAX_TELEOP_ROTATION * MathUtil.applyDeadband(-this.rightStickX.getAsDouble(), Drive.CONTROLLER_DEADBAND);

            if (vX == 0.0 && vY == 0.0 && omega == 0) {
                if (Utils.getSystemTimeSeconds() - startTime >= Drive.BRAKE_IDLE_TIME) {
                    drivetrain.setControl(xBrake);
                    return;
                }
            } else {
                startTime = Utils.getSystemTimeSeconds();
            }

            if (Optional.of(Alliance.Red).equals(Util.getAlliance())) {
                vX = -1 * vX;
                vY = -1 * vY;
            }
            drivetrain.setControl(this.drive
                    .withVelocityX(vX)
                    .withVelocityY(vY)
                    .withRotationalRate(omega));
        })).withName("regular drive");
    }

    public Command teleopDrive() {
        return drive(1);
    }

    public Command shootingTeleopDrive() {
        return drive(Drive.TELEOP_SHOOTING_VELOCITY_MULTIPLIER);
    }

    public Command stop() {
        return this.runOnce(() -> drivetrain.setControl(this.drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)));
    }

    /**
     * Follows the given field-centric path sample with PID and applies any
     * velocities as feed forwards.
     *
     * @param sample Provides sample to execute.
     */
    public void followSample(SwerveSample sample) {
        Pose2d currPose = inputs.Pose;
        Pose2d target = sample.getPose();

        this.autonTargetPose = target;

        double currentTime = Utils.getCurrentTimeSeconds();

        double vx = 0.0, vy = 0.0, omega = 0.0;
        vx = sample.vx + this.autonXController.calculate(currPose.getX(), target.getX(), currentTime);
        vy = sample.vy + this.autonYController.calculate(currPose.getY(), target.getY(), currentTime);
        omega = sample.omega + this.autonHeadingController.calculate(currPose.getRotation().getRadians(),
                target.getRotation().getRadians(), currentTime);

        drivetrain.setControl(this.auton
                .withRotationalRate(omega)
                .withVelocityX(vx)
                .withVelocityY(vy));
    }
}
