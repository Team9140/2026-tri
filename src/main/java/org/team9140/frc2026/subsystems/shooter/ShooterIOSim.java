package org.team9140.frc2026.subsystems.shooter;

import org.team9140.frc2026.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private final DCMotor motor;
    private final DCMotorSim shooterMotorSim;
    private boolean coastingOut;
    private double goal;

    public ShooterIOSim() {
        motor = DCMotor.getKrakenX60Foc(2);
        shooterMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.01, 1),
            motor);
        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double appliedVoltage = 0.0;
        double shooterVelocity = shooterMotorSim.getAngularVelocityRPM() / 60.0;
        if (!(DriverStation.isDisabled() || coastingOut)) {
            if (shooterVelocity < goal) {
                appliedVoltage = 12;
            }
        }
        shooterMotorSim.setInputVoltage(appliedVoltage);

        inputs.connected = true;
        inputs.velocityRotsPerSec = shooterVelocity;
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = shooterMotorSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motor.getCurrent(shooterMotorSim.getTorqueNewtonMeters());
        inputs.tempCelsius = 0.0;

        inputs.followerConnected = true;
        inputs.followerAppliedVoltage = appliedVoltage;
        inputs.followerSupplyCurrentAmps = shooterMotorSim.getCurrentDrawAmps();
        inputs.followerTorqueCurrentAmps = motor.getCurrent(shooterMotorSim.getTorqueNewtonMeters());
        inputs.followerTempCelsius = 0.0;

        shooterMotorSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void goToVelocity(double velocityRotsPerSec) {
        coastingOut = false;
        goal = velocityRotsPerSec;
    }

    @Override
    public void off() {
        coastingOut = true;
    }
}
