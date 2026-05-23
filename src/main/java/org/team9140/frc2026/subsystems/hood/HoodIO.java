package org.team9140.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    static class HoodIOInputs {
        // motor logging
        public double hoodAngleRotations;
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
        public boolean connected;

        // CANcoder logging
        public double CANcoderAbsolutePositionRot;
        public double CANcoderPositionRot;
        public boolean CANcoderConnected;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void moveToPosition(double angleRotations) {}
    
}
