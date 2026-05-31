package org.team9140.frc2026.commands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.subsystems.drive.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.extender.Extender;
import org.team9140.frc2026.subsystems.feeder.Feeder;
import org.team9140.frc2026.subsystems.hood.Hood;
import org.team9140.frc2026.subsystems.roller.Roller;
import org.team9140.frc2026.subsystems.shooter.Shooter;
import org.team9140.frc2026.subsystems.spinner.Spinner;
import org.team9140.frc2026.subsystems.turret.Turret;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutonomousRoutines {
    private CommandSwerveDrivetrain drivetrain;
    private Turret turret;
    private Shooter shooter;
    private Hood hood;
    private Feeder feeder;
    private Spinner spinner;
    private Extender extender;
    private Roller roller;

    private final Trigger readyToShoot;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain,
                 Turret turret,
                 Shooter shooter,
                 Hood hood,
                 Feeder feeder,
                 Spinner spinner,
                 Extender extender,
                 Roller roller) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.spinner = spinner;
        this.extender = extender;
        this.roller = roller;

        Command resetAllSubsystems = Commands.parallel(
            this.drivetrain.stop(),
            this.turret.off(),
            this.shooter.off(),
            this.hood.off(),
            this.feeder.reverseAndOff(),
            this.spinner.off(),
            this.roller.off()
        );

        this.readyToShoot = turret.atPosition.and(hood.atPosition).and(shooter.atVelocity);

        this.autoFactory = new AutoFactory(this.drivetrain::getDrivetrainPose,
            this.drivetrain::resetPose, 
            this.drivetrain::followSample, 
            true, 
            this.drivetrain, 
            (Trajectory<SwerveSample> traj, Boolean end) -> {
                // Documentation doesn't say if end=true means traj starts or traj ends so this is a guess
                Logger.recordOutput("Current trajectory is starting", end);
                Logger.recordOutput("Current trajectory name", traj.name());
            }
            );
        
        this.autoChooser = new AutoChooser("Do Nothing");

        autoChooser.addCmd("Shoot Preload", this::shootPreload);
        autoChooser.addRoutine("One Pass Over Bump From Left", this::onePassBumpDepot);

        SmartDashboard.putData("Auto Chooser", this.autoChooser);;
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        // Reset everything at the end of auto so no commands outlive auto
        // Default commands would probably get rid of the need for this
        RobotModeTriggers.autonomous().onFalse(resetAllSubsystems);
    }

    @AutoLogOutput
    private Pose2d initialAutoPose = null;

    private void setInitialAutoPose(Pose2d pose) {
        initialAutoPose = pose;
    }

    private Command shootPreload() {
        Command aimThenShootWhenReady = Commands.parallel(
            Commands.parallel(turret.aim(), shooter.aim(), hood.aim()), // aim everything
            Commands.sequence(
                Commands.waitUntil(readyToShoot),
                Commands.parallel(spinner.feed(), feeder.feed()))); // shoot
        Command stopAimingAndShooting = Commands.parallel(
            turret.off(),
            shooter.off(),
            hood.off(),
            spinner.off(),
            feeder.reverseAndOff()
        );
        return Commands.sequence(
            aimThenShootWhenReady.withTimeout(15),
            stopAimingAndShooting,
            this.extender.armOut()
        );
    }
    
    private AutoRoutine onePassBumpDepot() {
        Command aimThenShootWhenReady = Commands.parallel(
            Commands.parallel(turret.aim(), shooter.aim(), hood.aim()), // aim everything
            Commands.sequence(
                Commands.waitUntil(readyToShoot),
                Commands.parallel(spinner.feed(), feeder.feed()))); // shoot
        Command stopAimingAndShooting = Commands.parallel(
            turret.off(),
            shooter.off(),
            hood.off(),
            spinner.off(),
            feeder.reverseAndOff()
        );
        
        AutoRoutine routine = autoFactory.newRoutine("testRoutine");
        
        AutoTrajectory traj = routine.trajectory("Bump_Depot_Deep");
        routine.active().onTrue(
            Commands.sequence(
                extender.armOut(),
                roller.intake(),
                traj.cmd()
            )
        );

        traj.done().onTrue(
            Commands.sequence(
                aimThenShootWhenReady.withTimeout(5),
                stopAimingAndShooting
        ));

        traj.getInitialPose().ifPresent(this::setInitialAutoPose);

        return routine;
    }
}
