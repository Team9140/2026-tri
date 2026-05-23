package org.team9140.frc2026.subsystems.spinner;

import org.team9140.frc2026.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpinnerIOSim implements SpinnerIO{
    private final DCMotor motor;
    private final DCMotorSim spinnerSim;

    public SpinnerIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        spinnerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1), 
            motor);
    }

    @Override
    public void updateInputs(SpinnerIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            off();
        }

        inputs.connected = true;
        inputs.appliedVoltage = spinnerSim.getInputVoltage();
        inputs.supplyCurrentAmps = spinnerSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motor.getCurrent(spinnerSim.getTorqueNewtonMeters());
        inputs.tempCelsius = 0.0;

        spinnerSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void runVoltage(double voltage) {
        spinnerSim.setInputVoltage(voltage);
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
