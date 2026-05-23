package org.team9140.frc2026.subsystems.extender;


import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {

    @AutoLog
    static class ExtenderIOInputs {
        public double intakePosition;
        public double motorPosition;
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;
    }

    default void updateInputs(ExtenderIOInputs inputs) {};
    
    default void goToPosition(double position, double maxVelocity) {};

}
