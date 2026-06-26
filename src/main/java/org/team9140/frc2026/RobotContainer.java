// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.commands.AutonomousRoutines;
import org.team9140.frc2026.generated.TunerConstants;
import org.team9140.frc2026.subsystems.drive.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.drive.SwerveDriveIO;
import org.team9140.frc2026.subsystems.extender.Extender;
import org.team9140.frc2026.subsystems.extender.ExtenderIO;
import org.team9140.frc2026.subsystems.extender.ExtenderIOReal;
import org.team9140.frc2026.subsystems.extender.ExtenderIOSim;
import org.team9140.frc2026.subsystems.feeder.Feeder;
import org.team9140.frc2026.subsystems.feeder.FeederIO;
import org.team9140.frc2026.subsystems.feeder.FeederIOReal;
import org.team9140.frc2026.subsystems.hood.Hood;
import org.team9140.frc2026.subsystems.hood.HoodIO;
import org.team9140.frc2026.subsystems.hood.HoodIOReal;
import org.team9140.frc2026.subsystems.hood.HoodIOSim;
import org.team9140.frc2026.subsystems.roller.Roller;
import org.team9140.frc2026.subsystems.roller.RollerIO;
import org.team9140.frc2026.subsystems.roller.RollerIOReal;
import org.team9140.frc2026.subsystems.shooter.Shooter;
import org.team9140.frc2026.subsystems.shooter.ShooterIO;
import org.team9140.frc2026.subsystems.shooter.ShooterIOReal;
import org.team9140.frc2026.subsystems.shooter.ShooterIOSim;
import org.team9140.frc2026.subsystems.spinner.Spinner;
import org.team9140.frc2026.subsystems.spinner.SpinnerIO;
import org.team9140.frc2026.subsystems.spinner.SpinnerIOReal;
import org.team9140.frc2026.subsystems.turret.Turret;
import org.team9140.frc2026.subsystems.turret.TurretIO;
import org.team9140.frc2026.subsystems.turret.TurretIOReal;
import org.team9140.frc2026.subsystems.turret.TurretIOSim;
import org.team9140.frc2026.subsystems.vision.Vision;
import org.team9140.frc2026.subsystems.vision.VisionConstants;
import org.team9140.frc2026.subsystems.vision.VisionIO;
import org.team9140.frc2026.subsystems.vision.VisionIOLimelight;
import org.team9140.frc2026.subsystems.vision.VisionIOPhotonVisionSim;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  private final Roller roller;
  private final Extender extender;
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Spinner spinner;
  private final Feeder feeder;
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;
  private final AutonomousRoutines autos;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        drivetrain = new CommandSwerveDrivetrain(TunerConstants.createRealDrivetrain());
        vision = new Vision(drivetrain::addVisionMeasurement,
            new VisionIOLimelight(VisionConstants.camera0Name, drivetrain::getRotation),
            new VisionIOLimelight(VisionConstants.camera1Name, drivetrain::getRotation),
            new VisionIOLimelight(VisionConstants.camera2Name, drivetrain::getRotation));
        roller = new Roller(new RollerIOReal());
        extender = new Extender(new ExtenderIOReal());
        turret = new Turret(new TurretIOReal());
        hood = new Hood(new HoodIOReal());
        shooter = new Shooter(new ShooterIOReal());
        spinner = new Spinner(new SpinnerIOReal());
        feeder = new Feeder(new FeederIOReal());
      }
      case SIM -> {
        drivetrain = new CommandSwerveDrivetrain(TunerConstants.createSimDrivetrain());
        vision = new Vision(
            drivetrain::addVisionMeasurement,
            new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0,
                drivetrain::getDrivetrainPose),
            new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1,
                drivetrain::getDrivetrainPose),
            new VisionIOPhotonVisionSim(VisionConstants.camera2Name, VisionConstants.robotToCamera2,
                drivetrain::getDrivetrainPose));
        roller = new Roller(new RollerIOReal());
        extender = new Extender(new ExtenderIOSim());
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        spinner = new Spinner(new SpinnerIOReal());
        feeder = new Feeder(new FeederIOReal());
      }
      default -> { // This is replay but we need a default case for it to work
        drivetrain = new CommandSwerveDrivetrain(new SwerveDriveIO() {
        });
        vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO[3]);
        roller = new Roller(new RollerIO() {
        });
        extender = new Extender(new ExtenderIO() {
        });
        turret = new Turret(new TurretIO() {
        });
        hood = new Hood(new HoodIO() {
        });
        shooter = new Shooter(new ShooterIO() {
        });
        spinner = new Spinner(new SpinnerIO() {
        });
        feeder = new Feeder(new FeederIO() {
        });
      }
    }

    // Set up subsystems
    turret.setRobotDataSuppliers(drivetrain::getDrivetrainPose, drivetrain::getDrivetrainSpeeds);
    shooter.setRobotDataSuppliers(drivetrain::getDrivetrainPose, drivetrain::getDrivetrainSpeeds);
    hood.setRobotDataSuppliers(drivetrain::getDrivetrainPose, drivetrain::getDrivetrainSpeeds);
    autos = new AutonomousRoutines(drivetrain, turret, shooter, hood, feeder, spinner, extender, roller);
    configureBindings(); // Configure the trigger bindings
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private final CommandXboxController controller = new CommandXboxController(0);

  private void configureBindings() {
    Command normalDriveCommand = drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
        controller::getRightX);
    drivetrain.setDefaultCommand(normalDriveCommand);

    Trigger readyToShoot = hood.atPosition.and(shooter.atVelocity).and(turret.atPosition);
    Trigger wantAim = this.controller.rightTrigger(0.3).debounce(Constants.Turret.TURN_SHOOTER_OFF_TIME,
        DebounceType.kFalling);
    Trigger wantShoot = this.controller.rightTrigger(0.9);

    Command aimOnCommand = turret.aim().alongWith(shooter.aim()).alongWith(hood.aim());
    Command aimOffCommand = turret.off().alongWith(shooter.off()).alongWith(hood.off());
    wantAim.onTrue(aimOnCommand).onFalse(aimOffCommand);
    wantAim.whileTrue(drivetrain.shootingDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

    Command shootOnCommand = spinner.feed().alongWith(feeder.feed());
    Command shootOffCommand = spinner.off().alongWith(feeder.reverseAndOff());
    wantShoot.and(readyToShoot).onTrue(shootOnCommand).onFalse(shootOffCommand);

    Trigger wantIntake = this.controller.rightBumper();
    Trigger wantSqueeze = this.controller.leftBumper();

    wantIntake.and(wantSqueeze.negate()).onTrue(extender.armOut().alongWith(roller.intake()))
        .onFalse(roller.off());
    wantSqueeze.onTrue(extender.armIn().alongWith(roller.intake()))
        .onFalse(roller.off());

    // I don't think there's replacement for entering numbers yet so still using
    // smart dashboard
    SmartDashboard.putNumber("tuning RPM", 2500);
    SmartDashboard.putNumber("tuning Angle", 24.0);
    Command tuningCommand = turret.off()
        .alongWith(shooter.tuning(() -> SmartDashboard.getNumber("tuning RPM", 2500)))
        .alongWith(hood.tuning(() -> SmartDashboard.getNumber("tuning Angle", 24.0)));
    this.controller.y().onTrue(tuningCommand).onFalse(aimOffCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * This is now not needed as the choreo auto factory just has us bind the auto
   * command to RobotModeTriggers.autonomous()
   * public Command getAutonomousCommand() {
   * return null;
   * }
   */

  public void updateViz() {

    Pose3d intakePose = new Pose3d(
        extender.getPosition() * Math.cos(Units.degreesToRadians(-6.689)),
        0,
        extender.getPosition() * Math.sin(Units.degreesToRadians(-6.689)),
        new Rotation3d());

    Pose3d turretPose = new Pose3d(Constants.Turret.POSITION_TO_ROBOT.getX(), Constants.Turret.POSITION_TO_ROBOT.getY(),
        0.345,
        new Rotation3d(0, 0, Units.rotationsToRadians(turret.getPosition())));

    Pose3d hoodPose = turretPose.transformBy(new Transform3d(Constants.Turret.TURRET_AXIS_TO_FLYWHEEL_AXIS,
        new Rotation3d(0, -Units.rotationsToRadians(hood.getPosition()) - Units.degreesToRadians(18), 0)));

    Logger.recordOutput("ComponentsPoseArray", new Pose3d[] {
        intakePose,
        turretPose,
        hoodPose });
  }
}
