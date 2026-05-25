package org.team9140.frc2026.subsystems.roller;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim extends RollerIOReal {
    private final DCMotorSim rollerSim;
    private final DCMotor motor;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = 0.004;

    public RollerIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1),
            motor);
        
        rollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        
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
        double rollerVolts = this.rollerMotor.getSimState().getMotorVoltage();
        this.rollerSim.setInputVoltage(rollerVolts);
        this.rollerSim.update(dt);
        // update motor position
        this.rollerMotor.getSimState().setRawRotorPosition(this.rollerSim.getAngularPositionRotations());
        this.rollerMotor.getSimState().setRotorVelocity(this.rollerSim.getAngularVelocityRPM() / 60.0);
        this.rollerMotor.getSimState().setRotorAcceleration(
                this.rollerSim.getAngularAccelerationRadPerSecSq() / 2.0 / Math.PI);
    }
}
