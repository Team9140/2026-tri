package org.team9140.frc2026.subsystems.spinner;

import org.team9140.frc2026.Constants;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpinnerIOSim extends SpinnerIOReal{
    private final DCMotor motor;
    private final DCMotorSim spinnerSim;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = Constants.SIM_LOOP_PERIOD;

    public SpinnerIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        spinnerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1), 
            motor);
        spinnerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        
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
        double spinnerMotorVolts = this.spinnerMotor.getSimState().getMotorVoltage();
        this.spinnerSim.setInputVoltage(spinnerMotorVolts);
        this.spinnerSim.update(dt);
        this.spinnerMotor.getSimState().setRawRotorPosition(this.spinnerSim.getAngularPositionRotations());
        this.spinnerMotor.getSimState().setRotorVelocity(this.spinnerSim.getAngularVelocityRPM() / 60.0);
        this.spinnerMotor.getSimState().setRotorAcceleration(
                this.spinnerSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
    }
}
