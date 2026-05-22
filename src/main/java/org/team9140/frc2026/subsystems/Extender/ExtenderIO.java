package org.team9140.frc2026.subsystems.extender;


import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {

    @AutoLog
    static class ExtenderIOInputs {
        public double angle;
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;
    }

    default void updateInputs(ExtenderIOInputs inputs) {};
    
    default void setPosition(double position, double maxVelocity) {};

    default double getPosition() { return 0.0; };
}
