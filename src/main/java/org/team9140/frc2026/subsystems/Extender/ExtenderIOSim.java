package org.team9140.frc2026.subsystems.extender;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ExtenderIOSim implements ExtenderIO {
    private final ElevatorSim extenderSim;
    private final DCMotor motor;
    
    private final ProfiledPIDController pidController;
    
    private double appliedVolts = 0.0;

    public ExtenderIOSim() {
        motor = DCMotor.getKrakenX44(1);
        extenderSim = new ElevatorSim(motor,
            Constants.Extender.GEAR_RATIO,
            10,
            Constants.Extender.PINION_CIRCUMFERENCE / Math.PI / 2.0,
            Constants.Extender.REVERSE_SOFT_LIMIT_THRESHOLD,
            Constants.Extender.ARM_OUT_POSITION,
            false,
            0);

        pidController = new ProfiledPIDController(
            Constants.Extender.KP, 
            0.0, 
            0.0, 
            new TrapezoidProfile.Constraints(
                Constants.Extender.ARM_OUT_VELOCITY, 
                Constants.Extender.MM_ACCELERATION
            ), 
            Constants.LOOP_PERIOD
        );
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        double intakePos = extenderSim.getPositionMeters();

        appliedVolts = Util.clamp(pidController.calculate(intakePos / Constants.Extender.PINION_CIRCUMFERENCE), 12.0);
        extenderSim.setInputVoltage(appliedVolts);

        inputs.connected = true;
        inputs.motorPosition = intakePos / Constants.Extender.PINION_CIRCUMFERENCE;
        inputs.intakePosition = intakePos;
        inputs.appliedVoltage = appliedVolts;
        inputs.supplyCurrentAmps = extenderSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = 0; // I don't know what to do for this one
        inputs.tempCelsius = 0.0;
        

        extenderSim.update(Constants.LOOP_PERIOD);
    }

    @Override
    public void goToPosition(double position, double maxVelocity) {
        this.pidController.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, Constants.Extender.MM_ACCELERATION));
        this.pidController.setGoal(position / Constants.Extender.PINION_CIRCUMFERENCE);
    }

    @Override
    public double getPosition() {
        return this.extenderSim.getPositionMeters();
    }
}
