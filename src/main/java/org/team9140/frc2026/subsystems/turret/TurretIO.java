package org.team9140.frc2026.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    
    @AutoLog
    static class TurretIOInputs {
        // motor logging
        public double turretAngleRotations;
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

    default void updateInputs(TurretIOInputs inputs) {};
    
    // moves turret to a specific rotation NOT angle, gearing stuff will be handled inside func
    // this is because the phonix 6 stuff automatically handles the sensor to mechanism stuff
    default void moveToPosition(double positionRot) {};

    default void runVoltage(double voltage) {};

    default void brake() {};
}
