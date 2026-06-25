package org.team9140.frc2026.subsystems.hood;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Hood;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim extends HoodIOReal{
    private final DCMotor motor;
    private final SingleJointedArmSim hoodMotorSim;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = Constants.SIM_LOOP_PERIOD;
    
    public HoodIOSim() {
        motor = DCMotor.getKrakenX44Foc(1);
        hoodMotorSim = new SingleJointedArmSim(
            motor,
            Hood.GEAR_RATIO,
            0.022,
            0.2,
            0,
            Math.PI / 2.0,
            false,
            0);

        hoodMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        hoodCANcoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        hoodCANcoder.getSimState().SensorOffset = Hood.CANCODER_OFFSET_ROTS;

        m_lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime);
        });

        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void updateSimState(double dt) {
        double hoodMotorVolts = hoodMotor.getSimState().getMotorVoltage();
        hoodMotorVolts -= hoodMotorSim.getVelocityRadPerSec() * 0.7;
        
        hoodMotorSim.setInputVoltage(hoodMotorVolts);
        hoodMotorSim.update(dt);

        hoodMotor.getSimState().setRawRotorPosition(
                hoodMotorSim.getAngleRads() * Hood.GEAR_RATIO / 2.0 / Math.PI);
        hoodMotor.getSimState().setRotorVelocity(
                hoodMotorSim.getVelocityRadPerSec() * Hood.GEAR_RATIO / 2.0 / Math.PI);
        
        hoodCANcoder.getSimState().setRawPosition(
            hoodMotorSim.getAngleRads() / 2.0 / Math.PI * Hood.SENSOR_TO_MECHANISM_RATIO);
        hoodCANcoder.getSimState().setVelocity(
            hoodMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI * Hood.SENSOR_TO_MECHANISM_RATIO);
    }
}
