package org.team9140.frc2026.subsystems.hood;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Hood;
import org.team9140.lib.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO{
    private final DCMotor motor;
    private final SingleJointedArmSim hoodMotorSim;
    private final ProfiledPIDController controller;
    
    public HoodIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        hoodMotorSim = new SingleJointedArmSim(
            motor,
            60,
            1,
            0.2,
            0,
            Math.PI / 2.0,
            false,
            0);

        controller = new ProfiledPIDController(
                Hood.SIM_KP,
                Hood.SIM_KI,
                Hood.SIM_KD,
                new TrapezoidProfile.Constraints(Hood.MM_CRUISE_VELOCITY, Hood.MM_ACCELERATION),
                Constants.LOOP_PERIOD
        );
        controller.setGoal(0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        double hoodAngle = hoodMotorSim.getAngleRads() / 2 / Math.PI;
        double appliedVoltage = 0.0;
        if (DriverStation.isDisabled()) {
            appliedVoltage = 0.0;
        }
        else {
            appliedVoltage = Util.clamp(controller.calculate(hoodAngle), 12);
        }
        hoodMotorSim.setInputVoltage(appliedVoltage);

        inputs.connected = true;
        inputs.appliedVoltage = appliedVoltage;
        inputs.hoodAngleRotations = hoodAngle;
        inputs.supplyCurrentAmps = hoodMotorSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = 0.0; // idk how to get this from sim
        inputs.tempCelsius = 0.0;

        // Not simulating the CANcoder
        inputs.CANcoderAbsolutePositionRot = 0.0;
        inputs.CANcoderPositionRot = 0.0;
        inputs.CANcoderConnected = false;

        hoodMotorSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void moveToPosition(double angleRotations) {
        controller.setGoal(angleRotations);
    }
}
