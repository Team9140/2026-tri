package org.team9140.frc2026.subsystems.feeder;

import org.team9140.frc2026.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO{
    private final DCMotor motor;
    private final DCMotorSim feederSim;

    public FeederIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1),
            motor);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            off();
        }

        inputs.connected = true;
        inputs.appliedVoltage = feederSim.getInputVoltage();
        inputs.supplyCurrentAmps = feederSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motor.getCurrent(feederSim.getTorqueNewtonMeters());
        inputs.tempCelsius = 0.0;

        feederSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void runVoltage(double voltage) {
        feederSim.setInputVoltage(voltage);
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
