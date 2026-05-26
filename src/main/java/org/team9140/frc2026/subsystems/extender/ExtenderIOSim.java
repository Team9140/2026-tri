package org.team9140.frc2026.subsystems.extender;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Extender;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ExtenderIOSim extends ExtenderIOReal{
    private final ElevatorSim extenderSim;
    private final DCMotor motor;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = Constants.SIM_LOOP_PERIOD;


    public ExtenderIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        extenderSim = new ElevatorSim(motor,
            Extender.GEAR_RATIO,
            1,
            Extender.PINION_CIRCUMFERENCE / Math.PI / 2.0,
            Extender.REVERSE_SOFT_LIMIT_THRESHOLD,
            Extender.ARM_OUT_POSITION,
            false,
            0);
        
        this.extenderMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

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
        double extendVolts = this.extenderMotor.getSimState().getMotorVoltage();
        this.extenderSim.setInputVoltage(extendVolts);
        this.extenderSim.update(dt);

        double pos = this.extenderSim.getPositionMeters();
        double vel = this.extenderSim.getVelocityMetersPerSecond();

        this.extenderMotor.getSimState().setRawRotorPosition(
                pos / Extender.PINION_CIRCUMFERENCE * Extender.GEAR_RATIO);
        this.extenderMotor.getSimState()
                .setRotorVelocity(vel / Extender.PINION_CIRCUMFERENCE
                        * Extender.GEAR_RATIO);
    }
}
