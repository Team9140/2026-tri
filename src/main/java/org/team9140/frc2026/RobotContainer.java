// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

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
import org.team9140.frc2026.subsystems.feeder.FeederIOSim;
import org.team9140.frc2026.subsystems.hood.Hood;
import org.team9140.frc2026.subsystems.hood.HoodIO;
import org.team9140.frc2026.subsystems.hood.HoodIOReal;
import org.team9140.frc2026.subsystems.hood.HoodIOSim;
import org.team9140.frc2026.subsystems.roller.Roller;
import org.team9140.frc2026.subsystems.roller.RollerIO;
import org.team9140.frc2026.subsystems.roller.RollerIOReal;
import org.team9140.frc2026.subsystems.roller.RollerIOSim;
import org.team9140.frc2026.subsystems.shooter.Shooter;
import org.team9140.frc2026.subsystems.shooter.ShooterIO;
import org.team9140.frc2026.subsystems.shooter.ShooterIOReal;
import org.team9140.frc2026.subsystems.shooter.ShooterIOSim;
import org.team9140.frc2026.subsystems.spinner.Spinner;
import org.team9140.frc2026.subsystems.spinner.SpinnerIO;
import org.team9140.frc2026.subsystems.spinner.SpinnerIOReal;
import org.team9140.frc2026.subsystems.spinner.SpinnerIOSim;
import org.team9140.frc2026.subsystems.turret.Turret;
import org.team9140.frc2026.subsystems.turret.TurretIO;
import org.team9140.frc2026.subsystems.turret.TurretIOReal;
import org.team9140.frc2026.subsystems.turret.TurretIOSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(0);

  private final Roller roller;
  private final Extender extender;
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Spinner spinner;
  private final Feeder feeder;
  private final CommandSwerveDrivetrain drivetrain;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        roller = new Roller(new RollerIOReal());
        extender = new Extender(new ExtenderIOReal());
        turret = new Turret(new TurretIOReal());
        hood = new Hood(new HoodIOReal());
        shooter = new Shooter(new ShooterIOReal());
        spinner = new Spinner(new SpinnerIOReal());
        feeder = new Feeder(new FeederIOReal());
        drivetrain = new CommandSwerveDrivetrain(TunerConstants.createRealDrivetrain());
      }
      case SIM -> {
        roller = new Roller(new RollerIOSim());
        extender = new Extender(new ExtenderIOSim());
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        spinner = new Spinner(new SpinnerIOSim());
        feeder = new Feeder(new FeederIOSim());
        drivetrain = new CommandSwerveDrivetrain(TunerConstants.createSimDrivetrain());
      }
      default -> { // This is replay but we need a default case for it to work
        roller = new Roller(new RollerIO() {});
        extender = new Extender(new ExtenderIO() {});
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spinner = new Spinner(new SpinnerIO() {});
        feeder = new Feeder(new FeederIO() {});
        drivetrain = new CommandSwerveDrivetrain(new SwerveDriveIO() {});
      }

    }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setJoystickInput(controller::getLeftX, controller::getLeftY, controller::getRightX);
    drivetrain.setDefaultCommand(drivetrain.teleopDrive());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
