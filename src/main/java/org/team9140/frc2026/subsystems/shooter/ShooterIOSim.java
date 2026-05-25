package org.team9140.frc2026.subsystems.shooter;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Shooter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim extends ShooterIOReal {
    private final DCMotor simMotor;
    private final DCMotorSim shooterMotorSim;
    private Notifier simNotifier;
    private double m_lastSimTime;
    private double kSimLoopPeriod = Constants.SIM_LOOP_PERIOD;

    public ShooterIOSim() {
        super(); // Sets up config stuff
        simMotor = DCMotor.getKrakenX60Foc(2);
        shooterMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(simMotor, 0.0024, 1),
            simMotor);
        shooterMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        
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
        double shooterSimVolts = shooterMotor.getSimState().getMotorVoltage();
        shooterMotorSim.setInputVoltage(shooterSimVolts);

        shooterMotorSim.update(dt);

        shooterMotor.getSimState().setRawRotorPosition(
                shooterMotorSim.getAngularPositionRotations() * Shooter.GEAR_RATIO);
        shooterMotor.getSimState().setRotorVelocity(
                shooterMotorSim.getAngularVelocityRPM() / 60.0 * Shooter.GEAR_RATIO);
        shooterMotor.getSimState().setRotorAcceleration(
                shooterMotorSim.getAngularAccelerationRadPerSecSq()
                        * Shooter.GEAR_RATIO / 2.0 / Math.PI);
    }
}
