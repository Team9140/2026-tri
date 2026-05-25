package org.team9140.frc2026.subsystems.feeder;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIOReal{
    private final DCMotor motor;
    private final DCMotorSim feederSim;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = 0.004;

    public FeederIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1),
            motor);
        
        feederMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

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
        double feederMotorVolts = this.feederMotor.getSimState().getMotorVoltage();
        this.feederSim.setInputVoltage(feederMotorVolts);
        this.feederSim.update(dt);
        this.feederMotor.getSimState().setRawRotorPosition(this.feederSim.getAngularPositionRotations());
        this.feederMotor.getSimState().setRotorVelocity(this.feederSim.getAngularVelocityRPM() / 60.0);
        this.feederMotor.getSimState().setRotorAcceleration(
                this.feederSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
    }
}
