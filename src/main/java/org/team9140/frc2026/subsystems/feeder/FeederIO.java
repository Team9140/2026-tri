package org.team9140.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    static class FeederIOInputs {
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;
    }

    default void updateInputs(FeederIOInputs inputs) {};
    
    default void runVoltage(double voltage) {};

    default void off() {};
}
