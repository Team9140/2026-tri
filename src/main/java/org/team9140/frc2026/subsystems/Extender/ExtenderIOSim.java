package org.team9140.frc2026.subsystems.Extender;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ExtenderIOSim implements ExtenderIO {
    private final DCMotorSim extenderSim;
    private final DCMotor motor;

    private double targetPosition;
    
    private final ProfiledPIDController pidController;
    
    private double appliedVolts = 0.0;

    public ExtenderIOSim() {
        motor = DCMotor.getKrakenX60Foc(1);
        extenderSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.0005, 1),
            motor);
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
        appliedVolts = Util.clamp(pidController.calculate(extenderSim.getAngularPositionRad()), 12.0);
        extenderSim.setInputVoltage(appliedVolts);

        extenderSim.update(Constants.LOOP_PERIOD);
        
        inputs.connected = true;
        inputs.angle = extenderSim.getAngularPositionRotations();
        inputs.appliedVoltage = extenderSim.getInputVoltage();
        inputs.supplyCurrentAmps = extenderSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motor.getCurrent(extenderSim.getTorqueNewtonMeters());
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setPosition(double position, double maxVelocity) {
        this.targetPosition = position;
        this.pidController.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, Constants.Extender.MM_ACCELERATION));
        this.pidController.setGoal(this.targetPosition / Constants.Extender.PINION_CIRCUMFERENCE);
    }

    @Override
    public double getPosition() {
        return this.extenderSim.getAngularPositionRotations() * Constants.Extender.PINION_CIRCUMFERENCE;
    }
}
