package org.team9140.frc2026.subsystems.canrange;

import org.littletonrobotics.junction.Logger;
import org.team9140.frc2026.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Canrange extends SubsystemBase {
    CanrangeIO canrange;
    CanrangeIOInputsAutoLogged inputs = new CanrangeIOInputsAutoLogged();

    public Canrange(CanrangeIO canrange) {
        this.canrange = canrange;
    }

    @Override
    public void periodic() {
        canrange.updateInputs(inputs);
        Logger.processInputs("Canrange", inputs);
    }
    
    public Trigger canrangeIsFull() {
        return new Trigger(() -> inputs.distance < Constants.Canrange.CANRANGE_FULL_THRESHOLD);
    }
}
