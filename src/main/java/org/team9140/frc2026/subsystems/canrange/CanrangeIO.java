package org.team9140.frc2026.subsystems.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CanrangeIO {
    
    @AutoLog
    static class CanrangeIOInputs {
        public double distance;
        public double signalStrength;
        public boolean connected;
    }

    default void updateInputs(CanrangeIOInputs inputs) {};
}
