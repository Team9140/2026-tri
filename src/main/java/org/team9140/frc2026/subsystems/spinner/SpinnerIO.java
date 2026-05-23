package org.team9140.frc2026.subsystems.spinner;

import org.littletonrobotics.junction.AutoLog;

public interface SpinnerIO {
    @AutoLog
    static class SpinnerIOInputs {
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;
    }

    default void updateInputs(SpinnerIOInputs inputs) {};
    
    default void runVoltage(double voltage) {};

    default void off() {};
}
