package org.team9140.frc2026.commands;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team9140.frc2026.Robot;
import org.team9140.frc2026.subsystems.drive.CommandSwerveDrivetrain;
import org.team9140.frc2026.subsystems.extender.Extender;
import org.team9140.frc2026.subsystems.feeder.Feeder;
import org.team9140.frc2026.subsystems.hood.Hood;
import org.team9140.frc2026.subsystems.roller.Roller;
import org.team9140.frc2026.subsystems.shooter.Shooter;
import org.team9140.frc2026.subsystems.spinner.Spinner;
import org.team9140.frc2026.subsystems.turret.Turret;
import org.team9140.lib.Util;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousRoutines {
    CommandSwerveDrivetrain drivetrain;
    Turret turret;
    Shooter shooter;
    Hood hood;
    Feeder feeder;
    Spinner spinner;
    Extender extender;
    Roller roller;

    enum Autos {
        TEST // example, should be deleted later
    }

    LoggedDashboardChooser<Autos> autoChooser = new LoggedDashboardChooser<>("Auto chooser");

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

        autoChooser.addOption("Testing", Autos.TEST);
    }

    private Command autonCommand = null;
    @AutoLogOutput
    private Pose2d initialAutoPose = null;

    // For each auto you have to do 3 things:
    // 1. load all needed trajectories
    // 2. set the intial pose
    // 3. create the actual sequence of commands
    private void updateAuto(Autos selectedAuto) {
        autonCommand = switch (selectedAuto) {
            case TEST -> {
                Trajectory<SwerveSample> traj = loadTrajectory("Bump_Depot_Deep");
                initialAutoPose = traj.getInitialPose(false).get();
                yield drivetrain.followTrajectory(traj)
                      .andThen(turret.aim());
            }
            default -> null;
        };
    }

    private Autos lastSelectedAuto;
    private DriverStation.Alliance lastAlliance;

    public Command getAutonomousCommand() {
        Autos selected = autoChooser.get();
        DriverStation.Alliance alliance = null;
        if (Util.getAlliance().isPresent()) {
            alliance = Util.getAlliance().get();
        }
        if (alliance != null && selected != null && (!alliance.equals(lastAlliance) || selected != lastSelectedAuto)) {
            updateAuto(selected);
            lastSelectedAuto = selected;
            lastAlliance = alliance;
        }
        if (Robot.isSimulation() && initialAutoPose != null) {
            drivetrain.resetPose(initialAutoPose);
        }
        return autonCommand;
    }

    private Trajectory<SwerveSample> loadTrajectory(String pathName) {
        Optional<Trajectory<SwerveSample>> traj = Choreo.<SwerveSample>loadTrajectory(pathName);
        // This should never be reached until the alliance is present
        DriverStation.Alliance alliance = Util.getAlliance().get();
        if (traj.isPresent()){
            return alliance.equals(Alliance.Blue) ? traj.get() : traj.get().flipped();
        }
        return null;
    }
}
