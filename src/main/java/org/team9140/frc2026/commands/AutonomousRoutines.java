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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        autoChooser.addRoutine("Left Pass Then Neutral", () -> onePass(true));
        autoChooser.addRoutine("Right Pass Then Neutral", () -> onePass(false));
        autoChooser.addRoutine("Middle To Depot Then Neutral", this::scoreDepot);

        SmartDashboard.putData("Auto Chooser", this.autoChooser);;
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        // Reset everything at the end of auto so no commands outlive auto
        Command resetAllSubsystems = Commands.parallel(
            this.drivetrain.stop(),
            this.turret.off(),
            this.shooter.off(),
            this.hood.off(),
            this.feeder.off(),
            this.spinner.off(),
            this.roller.off()
        );
        RobotModeTriggers.autonomous().onFalse(resetAllSubsystems);
    }

    @AutoLogOutput
    private Pose2d initialAutoPose = null;

    private Command aim() {
        return Commands.parallel(turret.aim(), shooter.aim(), hood.aim());
    }

    private Command shoot() {
        return Commands.parallel(spinner.feed(), feeder.feed());
    }

    private Command aimOff() {
        return Commands.parallel(turret.off(), shooter.off(), hood.off());
    }

    private Command shootOff() {
        return Commands.parallel(spinner.off(), feeder.reverseAndOff());
    }

    private Command aimAndShootOff() {
        return Commands.parallel(aimOff(), shootOff());
    }

    private Command waitTillAimed() {
        return Commands.waitUntil(readyToShoot);
    }

    //Shoots forever after aiming
    private Command aimThenShootWhenReady() {
        return Commands.parallel(aim(), Commands.sequence(waitTillAimed(), shoot()));
    }

    private Command shootPreload() {
        return Commands.sequence(
            aimThenShootWhenReady().withTimeout(15),
            aimAndShootOff(),
            this.extender.armOut()
        );
    }

    private AutoRoutine onePass(boolean onDepotSide) {
        AutoRoutine routine = this.autoFactory.newRoutine("One Pass Then Neutral");

        AutoTrajectory intakingTraj = routine.trajectory("depotNeutralPass");
        AutoTrajectory toNeutralTraj = routine.trajectory("depotShootingToNeutral");

        // flip traj if starting on other side
        if (!onDepotSide) {
            intakingTraj = intakingTraj.mirrorY();
            toNeutralTraj = toNeutralTraj.mirrorY();
        }
        // logs starting pose so it's visible on elastic and stuff
        this.initialAutoPose = intakingTraj.getInitialPose().get();

        // get fuel
        routine.active().onTrue(
            Commands.sequence(
                extender.armOut(), // No need for proxies bc these don't need to be
                roller.intake(),   // commanded during the traj
                intakingTraj.cmd())); 
        // There's a spawnCmd method which makes a command to schedule a separate traj command
        // That might solve requirements bleeding into the traj

        // shoot fuel
        intakingTraj.done().onTrue(
            Commands.sequence(
                drivetrain.stop(), // zeroes out any leftover velocity
                Commands.parallel(
                    new WaitCommand(7).andThen(extender.armIn()), // does the squeeze, maybe use canrange in future
                    aimThenShootWhenReady().withTimeout(13)),
                aimAndShootOff(),
                toNeutralTraj.cmd()));
        
        toNeutralTraj.done().onTrue(
            Commands.parallel(
                roller.off(),
                extender.armOut())); 

        return routine;
    }

    private AutoRoutine scoreDepot() {
        AutoRoutine routine = this.autoFactory.newRoutine("Score Depot Then Neutral");

        AutoTrajectory intakingTraj = routine.trajectory("getDepot");
        AutoTrajectory toNeutralTraj = routine.trajectory("depotCornerToNeutral");

        this.initialAutoPose = intakingTraj.getInitialPose().get();

        routine.active().onTrue(Commands.sequence(
                extender.armOutForDepot(), // No need for proxies bc these don't need to be
                roller.intake(),           // commanded during the traj
                intakingTraj.cmd()));
        
        intakingTraj.done().onTrue(Commands.sequence(
                drivetrain.stop(), // zeroes out any leftover velocity
                Commands.parallel(
                    new WaitCommand(7).andThen(extender.armIn()), // does the squeeze, maybe use canrange in future
                    aimThenShootWhenReady().withTimeout(10)),
                aimAndShootOff(),
                toNeutralTraj.cmd()));
        
        toNeutralTraj.done().onTrue(
            Commands.parallel(
                roller.off(),
                extender.armOut())); 

        return routine;
    }
}
