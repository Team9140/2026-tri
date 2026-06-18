package org.team9140.frc2026.subsystems.canrange;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Ports;
import org.team9140.lib.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;

public class CanrangeIOReal implements CanrangeIO {
    protected final CANrange canrange;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Double> signalStrength;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public CanrangeIOReal() {
        this.canrange = new CANrange(Ports.CANRANGE);

        ToFParamsConfigs tofConfigs = new ToFParamsConfigs()
            .withUpdateMode(UpdateModeValue.ShortRange100Hz);
        
        ProximityParamsConfigs proximityConfigs = new ProximityParamsConfigs()
            .withProximityThreshold(Constants.Canrange.CANRANGE_FULL_THRESHOLD);

        CANrangeConfiguration configs = new CANrangeConfiguration()
            .withToFParams(tofConfigs)
            .withProximityParams(proximityConfigs)
            .withFovParams(null);

        Util.tryUntilOk(() -> this.canrange.getConfigurator().apply(configs));

        this.distance = this.canrange.getDistance();
        this.signalStrength = this.canrange.getSignalStrength();

        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                distance,
                signalStrength)); 
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(this.canrange));
    }

    @Override
    public void updateInputs(CanrangeIOInputs inputs) {
        StatusCode canrangeStatus = BaseStatusSignal.refreshAll(
            distance, 
            signalStrength);
        inputs.connected = connectedDebouncer.calculate(canrangeStatus.isOK());
        inputs.distance = distance.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValue();
    }

}
