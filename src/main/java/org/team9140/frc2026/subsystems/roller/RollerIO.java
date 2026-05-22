package org.team9140.frc2026.subsystems.roller;


import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    static class RollerIOInputs {
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;
        public double followerAppliedVoltage;
        public double followerSupplyCurrentAmps;
        public double followerTorqueCurrentAmps;
        public double followerTempCelsius;
        public boolean followerConnected;
    }

    default void updateInputs(RollerIOInputs inputs) {};
    
    default void runVoltage(double voltage) {};

    default void off() {};


}
