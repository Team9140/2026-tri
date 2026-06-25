package org.team9140.frc2026.subsystems.turret;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Turret;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim extends TurretIOReal{
    private final SingleJointedArmSim turretMotorSim;
    private final DCMotor motor;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = Constants.SIM_LOOP_PERIOD;

    public TurretIOSim () {
        motor = DCMotor.getKrakenX44Foc(1);
        turretMotorSim = new SingleJointedArmSim(
                motor,
                Turret.GEAR_RATIO,
                0.094,
                0.2,
                -200 * Math.PI / 180.0,
                200 * Math.PI / 180.0,
                false,
                0);

        turretMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        turretCANcoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        turretCANcoder.getSimState().SensorOffset = Turret.CANCODER_OFFSET_ROTS;
        
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
        double turretSimVolts = turretMotor.getSimState().getMotorVoltage();
        // a fudge for friction damping out the oscillation
        turretSimVolts -= turretMotorSim.getVelocityRadPerSec() * 0.25;
        
        turretMotorSim.setInputVoltage(turretSimVolts);
        turretMotorSim.update(dt);
         
        turretMotor.getSimState().setRawRotorPosition(
                turretMotorSim.getAngleRads() * Turret.GEAR_RATIO / 2.0 / Math.PI);
        turretMotor.getSimState().setRotorVelocity(
                turretMotorSim.getVelocityRadPerSec() * Turret.GEAR_RATIO / 2.0 / Math.PI);
        turretCANcoder.getSimState().setRawPosition(
            turretMotorSim.getAngleRads() / 2.0 / Math.PI * Turret.SENSOR_TO_MECHANISM);
        turretCANcoder.getSimState().setVelocity(
            turretMotorSim.getVelocityRadPerSec() / 2.0 / Math.PI * Turret.SENSOR_TO_MECHANISM);
    }
}
