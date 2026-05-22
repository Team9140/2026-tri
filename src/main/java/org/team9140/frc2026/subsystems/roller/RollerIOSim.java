package org.team9140.frc2026.subsystems.roller;

import org.team9140.frc2026.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private final DCMotorSim rollerSim;
    private final DCMotor motor;

    public RollerIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1),
            motor);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {

        rollerSim.update(Constants.LOOP_PERIOD);

        inputs.connected = true;
        inputs.appliedVoltage = rollerSim.getInputVoltage();
        inputs.supplyCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motor.getCurrent(rollerSim.getTorqueNewtonMeters());
        inputs.tempCelsius = 0.0;

        // update follower motor inputs
        inputs.connected = true;
        inputs.followerAppliedVoltage = rollerSim.getInputVoltage();
        inputs.supplyCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.followerTorqueCurrentAmps = motor.getCurrent(rollerSim.getTorqueNewtonMeters());
        inputs.followerTempCelsius = 0.0;
    }

    @Override
    public void runVoltage(double voltage) {
        rollerSim.setInputVoltage(voltage);
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
