package org.team9140.frc2026.helpers;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.FieldConstants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class LookUpAimAlign {
    /*
     * Tags on hubs for LL filter: 2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27
     */
    // This is ported from the old code in case we need it

    private static InterpolatingDoubleTreeMap motorSpeedFromDistanceShooting = new InterpolatingDoubleTreeMap(); // in
                                                                                                                 // rotatations
                                                                                                                 // per
                                                                                                                 // second
    private static InterpolatingDoubleTreeMap hoodAngleFromDistanceShooting = new InterpolatingDoubleTreeMap(); // in
                                                                                                                // degrees
    private static InterpolatingDoubleTreeMap airtimeFromDistanceShooting = new InterpolatingDoubleTreeMap(); // in
                                                                                                              // seconds

    private static InterpolatingDoubleTreeMap motorSpeedFromDistancePassing = new InterpolatingDoubleTreeMap(); // in
                                                                            // rotations per second
    private static InterpolatingDoubleTreeMap hoodAngleFromDistancePassing = new InterpolatingDoubleTreeMap(); // in
                                                                                                               // degrees
    private static InterpolatingDoubleTreeMap airtimeFromDistancePassing;// = new InterpolatingDoubleTreeMap(); //in
                                                                         // seconds

    static {
        airtimeFromDistanceShooting.put(2.5, 1.0);
        airtimeFromDistanceShooting.put(2.7, 1.1);
        airtimeFromDistanceShooting.put(3.3, 1.1);
        airtimeFromDistanceShooting.put(4.0, 1.15);
        airtimeFromDistanceShooting.put(4.85, 1.2);
        airtimeFromDistanceShooting.put(6.0, 1.5);
        airtimeFromDistancePassing = airtimeFromDistanceShooting;

        double BUMPER_TO_TURRET_FT = 1.0;
        double HALF_HUB_FT = 2.0;
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(4 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1700.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(5 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1800.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(6 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1900.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(7 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(8 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(9 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(10 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2050.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(11 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2100.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(12 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2150.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(13 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2200.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(14 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2300.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(15 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2400.0 / 60.0);
        motorSpeedFromDistanceShooting.put(Units.feetToMeters(16 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2500.0 / 60.0);

        motorSpeedFromDistancePassing.put(Units.feetToMeters(1 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1400.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(2 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1500.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(3 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1600.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(4 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1700.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(5 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1800.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(6 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 1900.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(7 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(8 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(9 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2000.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(10 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2050.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(11 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2100.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(12 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2150.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(13 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2200.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(14 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2300.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(15 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2400.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(16 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2600.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(17 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 2800.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(18 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 3000.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(19 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 3100.0 / 60.0);
        motorSpeedFromDistancePassing.put(Units.feetToMeters(20 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 3200.0 / 60.0);

        hoodAngleFromDistanceShooting.put(Units.feetToMeters(4 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 21.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(5 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 21.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(6 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 21.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(7 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 23.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(8 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 26.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(9 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 29.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(10 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 31.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(11 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 32.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(12 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 33.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(13 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 33.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(14 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 34.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(15 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 35.0);
        hoodAngleFromDistanceShooting.put(Units.feetToMeters(16 + BUMPER_TO_TURRET_FT + HALF_HUB_FT), 37.0);

        hoodAngleFromDistancePassing.put(1.0, 42.0);
    }

    public static Translation2d getEffectivePose(Pose2d robotPose, Translation2d goalPose, ChassisSpeeds robotSpeed,
            boolean isPassing) {
        robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeed, robotPose.getRotation());
        Translation2d robotVelocity = new Translation2d(
                Turret.POSITION_TO_ROBOT.getX(),
                Turret.POSITION_TO_ROBOT.getY());
        robotVelocity = robotVelocity.rotateBy(robotPose.getRotation().plus(new Rotation2d(Math.PI / 2)))
                .times(-robotSpeed.omegaRadiansPerSecond);
        robotVelocity = robotVelocity.plus(new Translation2d(
                robotSpeed.vxMetersPerSecond,
                robotSpeed.vyMetersPerSecond));

        InterpolatingDoubleTreeMap airtimeLookup = isPassing ? airtimeFromDistancePassing : airtimeFromDistanceShooting;

        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(goalPose).getNorm();
        double airtime = airtimeLookup.get(distance);
        Translation2d newPose = goalPose.minus(robotVelocity.times(airtime));

        int iterations = 0;
        do {
            iterations++;
            distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(newPose).getNorm();
            airtime = airtimeLookup.get(distance);
            Logger.recordOutput("Estimated Airtime", airtime);
            Logger.recordOutput("Effective Pose", new Pose2d(newPose, new Rotation2d()));
        } while (newPose.minus(newPose = goalPose.minus(robotVelocity.times(airtime))).getNorm() > 0.05
                && iterations < 5);

        Logger.recordOutput("Number of iterations", iterations);

        return newPose;
    }

    public static double getRequiredSpeed(Pose2d robotPose, Translation2d effectivePose, boolean isPassing) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return isPassing ? motorSpeedFromDistancePassing.get(distance) : motorSpeedFromDistanceShooting.get(distance);
    }

    public static double getRequiredHoodAngle(Pose2d robotPose, Translation2d effectivePose, boolean isPassing) {
        double distance = robotPose.plus(Turret.POSITION_TO_ROBOT).getTranslation().minus(effectivePose).getNorm();
        return isPassing ? hoodAngleFromDistancePassing.get(distance) : hoodAngleFromDistanceShooting.get(distance);
    }

    public static double yawAngleToPos(Pose2d robotPose, Translation2d effectivePose) {
        effectivePose = (new Pose2d(effectivePose, Util.NOROTATION).relativeTo(robotPose)).getTranslation();
        return MathUtil.angleModulus(Math.atan2(
                (effectivePose.getY() - Turret.POSITION_TO_ROBOT.getY()),
                (effectivePose.getX() - Turret.POSITION_TO_ROBOT.getX()))
                - Turret.POSITION_TO_ROBOT.getRotation().getRadians());
    }

    public static Pose2d getZone(Pose2d robotPose) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();
        Pose2d position;
        if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())
                && rx > FieldConstants.Lines.RED_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.RED_CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Blue).equals(Util.getAlliance())
                && rx < FieldConstants.Lines.BLUE_ALLIANCE_ZONE) {
            position = FieldConstants.Hub.CENTER_POINT;
        } else if (Optional.of(DriverStation.Alliance.Red).equals(Util.getAlliance())) {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER_RED;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER_RED;
            }
        } else {
            if (ry < FieldConstants.FIELD_WIDTH / 2) {
                position = FieldConstants.FeedingPositions.FEEDING_POS_LOWER;
            } else {
                position = FieldConstants.FeedingPositions.FEEDING_POS_UPPER;
            }
        }
        return position;
    }

    public static Pose2d getHub() {
        if (Util.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
            return FieldConstants.Hub.RED_CENTER_POINT;
        }

        return FieldConstants.Hub.CENTER_POINT;
    }
}
