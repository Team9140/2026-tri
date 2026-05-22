// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2026;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    // running on actual robot
    REAL,
    // using physics sim
    SIM,
    // replaying from logged data
    REPLAY
  }

  public static final double LOOP_PERIOD = 0.02; // In seconds

  public static class Ports {
        public static final CANBus ROBO_RIO = CANBus.roboRIO();
        public static final CANBus CANIVORE = new CANBus("sixseven", "./logs/example.hoot");
        public static final CANBus SHOOTER_CANIVORE = new CANBus("kenny", "./logs/example.hoot");
        public static final int TURRET_MOTOR = 12;
        public static final int HOOD_MOTOR = 15;
        public static final int SHOOTER_MOTOR = 17;
        public static final int SHOOTER_FOLLOWER_MOTOR = 16;
        public static final int ROLLER_MOTOR = 26;
        public static final int ROLLER_FOLLOWER_MOTOR = 27;
        public static final int EXTENDER_MOTOR = 9;
        public static final int HOPPER_SPINNER_MOTOR = 10;
        public static final int HOPPER_FEEDER_MOTOR = 13;
        public static final int TURRET_CANCODER = 36;
        public static final int HOOD_CANCODER = 5;
    }

    public static class Drive {
        // TODO: Replace when drivetrain figured out
        //public static double MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.9;
        public static double MAX_TELEOP_ROTATION = Math.toRadians(360);

        //public static double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static double MIN_TELEOP_VELOCITY = 0.05;
        public static double MIN_TELEOP_ROTATION = Math.toRadians(2);
        public static double MIN_AUTON_VELOCITY = 0.05;
        public static double MIN_AUTON_ROTATION = Math.toRadians(2);

        public static final double X_CONTROLLER_P = 2.5 * 3.141592653589793238462643383279502884197169399375;
        public static final double X_CONTROLLER_I = 0.0;
        public static final double X_CONTROLLER_D = 0.015; // TODO: Raise value
        public static final double Y_CONTROLLER_P = X_CONTROLLER_P;
        public static final double Y_CONTROLLER_I = X_CONTROLLER_I;
        public static final double Y_CONTROLLER_D = X_CONTROLLER_D;
        public static final double HEADING_CONTROLLER_P = 8.0; // 8.0
        public static final double HEADING_CONTROLLER_I = 0.0;
        public static final double HEADING_CONTROLLER_D = 0.03; // 0.04

        public static final double REACHEDPOSE_DEBOUNCE = 0.5;
        public static final double BRAKE_IDLE_TIME = 1.0;
        public static final double TELEOP_SHOOTING_VELOCITY_MULTIPLIER = 0.67;
    }

    public static class Shooter {
        public static final double PEAK_FORWARD_TORQUE = 70.0;
        public static final double IDLE_VOLTAGE = 4.0;

        public static final double SHOOTER_KS = 0;
        public static final double SHOOTER_KV = 0;
        public static final double SHOOTER_KA = 0;
        public static final double SHOOTER_KP = 100;
        public static final double SHOOTER_KI = 0;
        public static final double SHOOTER_KD = 0;

        public static final double FLYWHEEL_GEAR_RATIO = 26.0 / 24.0;

        public static final double AUTO_IDLE_TIMESTAMP = 2;
    }

    public static class Turret {
        public static final double STATOR_CURRENT_LIMIT = 60.0;
        public static final double SUPPLY_CURRENT_LIMIT = 40.0;

        public static final double MM_ACCELERATION = 12;
        public static final double MM_CRUISE_VELOCITY = 8;

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 1000;
        public static final double KI = 0;
        public static final double KD = 50;

        public static final double ADJUST_VOLTAGE = 2.0;
        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 130.0 / 360.0; // rot
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = -150.0 / 360.0; // rot
        public static final double GEAR_RATIO = 90.0 / 10.0 * 50.0 / 11.0;
        public static final double SENSOR_TO_MECHANISM = 90.0 / 18.0;

        public static final double OVERTURN_LOOKAHEAD_TIME = 1.5;
        public static final Transform2d POSITION_TO_ROBOT = new Transform2d(-0.124, -0.159, new Rotation2d(-Math.PI));

        public static final double TURN_SHOOTER_OFF_TIME = 1.0;
        public static final double CANCODER_OFFSET_ROTS = 0.246;
    }

    public static class Hood {
        public static final double STATOR_CURRENT_LIMIT = 25.0;
        public static final double SUPPLY_CURRENT_LIMIT = 10.0;

        public static final double MM_ACCELERATION = 3.0;
        public static final double MM_CRUISE_VELOCITY = 4.0;

        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
        public static final double KP = 9140.0;
        public static final double KI = 0.0;
        public static final double KD = 67.0;

        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = 0.08; // 0.08;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0.01;
        public static final double GEAR_RATIO = 36.0 / 8.0 * 170.0 / 10.0;
        public static final double SENSOR_TO_MECHANISM_RATIO = 170.0 / 10.0 * 16.0 / 25.0;
        public static final double CANCODER_OFFSET_ROTS = -0.32;
        public static final double ANGLE_MIN = Units.degreesToRadians(18.0);
    }

    public static class Roller {
        public static final double INTAKING_ROLLER_VOLTAGE = 12.0;

        public static final double STATOR_CURRENT_LIMIT = 40.0;
        public static final double SUPPLY_CURRENT_LIMIT = 30.0;
    }

    public static class Extender {
        public static final double GEAR_RATIO = 36.0 / 15.0 * 50.0 / 11.0;

        public static final int PINION_TEETH = 10;
        public static final double PINION_DP = 10;
        public static final double PINION_CIRCUMFERENCE = Units.inchesToMeters(PINION_TEETH / PINION_DP * Math.PI);

        public static final double STATOR_CURRENT_LIMIT = 30.0;
        public static final double SUPPLY_CURRENT_LIMIT = 30.0;

        public static final double ARM_OUT_VELOCITY = 64;
        public static final double ARM_IN_VELOCITY = 32;
        public static final double MM_ACCELERATION = 64;
        public static final double KP = 500;

        public static final double ARM_OUT_POSITION = 0.29;
        public static final double ARM_IN_POSITION = ARM_OUT_POSITION / 2.0;
        public static final double DEPOT_ARM_OUT_POSITION = 0.24;

        public static final double FORWARD_SOFT_LIMIT_THRESHOLD = ARM_OUT_POSITION / PINION_CIRCUMFERENCE;
        public static final double REVERSE_SOFT_LIMIT_THRESHOLD = 0;
    }

    public static class Hopper {
        public static final double SPINNER_STATOR_CURRENT_LIMIT = 40.0;
        public static final double FEEDER_STATOR_CURRENT_LIMIT = 40.0;
        public static final double SPINNER_SUPPLY_CURRENT_LIMIT = 40;
        public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 60;

        public static final double SPINNER_VOLTAGE = 10.0;
        public static final double FEEDER_VOLTAGE = 12;
        public static final double SPINNER_REVERSE_VOLTAGE = -2.0;

        public static final double REVERSE_FEEDER_TIME = 1.0;
    }

    public static class Vision {
        public static final String[] CAMERA_NAMES = {"limelight-a", "limelight-b", "limelight-center"};
        public static Transform3d[] ROBOT_TO_CAM = {null, null, null};

        public static final double LEFT_CAMERA_X = -0.268;
        public static final double LEFT_CAMERA_Y = 0.350;
        public static final double LEFT_CAMERA_Z = 0.274;
        public static final double LEFT_CAMERA_YAW = 90;
        public static final double LEFT_CAMERA_PITCH = 0.0;
        public static final double LEFT_CAMERA_ROLL = 0.0;

        public static final double RIGHT_CAMERA_X = -0.268;
        public static final double RIGHT_CAMERA_Y = -0.350;
        public static final double RIGHT_CAMERA_Z = 0.274;
        public static final double RIGHT_CAMERA_YAW = -90;
        public static final double RIGHT_CAMERA_PITCH = 0.0;
        public static final double RIGHT_CAMERA_ROLL = 0.0;
    }
}
