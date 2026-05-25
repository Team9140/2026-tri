package org.team9140.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    static class ShooterIOInputs {
        public double velocityRotsPerSec;
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

    default void updateInputs(ShooterIOInputs inputs) {}

    default void goToVelocity(double velocityRotsPerSec) {}

    default void off() {}
}
