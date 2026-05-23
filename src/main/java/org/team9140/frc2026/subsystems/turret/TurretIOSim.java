package org.team9140.frc2026.subsystems.turret;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Turret;
import org.team9140.lib.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO{
    private final SingleJointedArmSim turretMotorSim;
    private final DCMotor motor;
    
    private final ProfiledPIDController controller;
    private boolean usingVoltage;
    private double requestedVoltage;

    public TurretIOSim () {
        motor = DCMotor.getKrakenX60Foc(1);
        turretMotorSim = new SingleJointedArmSim(
                motor,
                60,
                1,
                0.2,
                -200 * Math.PI / 180.0,
                200 * Math.PI / 180.0,
                false,
                0);
        
        controller = new ProfiledPIDController(
                Turret.SIM_KP,
                Turret.SIM_KI,
                Turret.SIM_KD,
                new TrapezoidProfile.Constraints(Turret.MM_CRUISE_VELOCITY, Turret.MM_ACCELERATION),
                Constants.LOOP_PERIOD
        );
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double turretPos = turretMotorSim.getAngleRads() / 2 / Math.PI;
        double appliedVolts = 0.0;
        if (!usingVoltage){
            appliedVolts = Util.clamp(controller.calculate(turretPos), 12);
        }
        else {
            appliedVolts = requestedVoltage;
        }
        turretMotorSim.setInputVoltage(appliedVolts);

        inputs.connected = true;
        inputs.appliedVoltage = appliedVolts;
        inputs.turretAngleRotations = turretPos;
        inputs.supplyCurrentAmps = turretMotorSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = 0.0; // idk how to get this from sim
        inputs.tempCelsius = 0.0;

        // Not simulating the CANcoder
        inputs.CANcoderAbsolutePositionRot = 0.0;
        inputs.CANcoderPositionRot = 0.0;
        inputs.CANcoderConnected = false;

        turretMotorSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void moveToPosition(double positionRot) {
        controller.setGoal(positionRot);
        usingVoltage = false;
    }

    @Override
    public void runVoltage(double voltage) {
        requestedVoltage = voltage;
        usingVoltage = true;
    }

    @Override
    public void brake() {
        runVoltage(0);
    }


}
