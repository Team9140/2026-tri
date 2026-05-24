package org.team9140.frc2026.subsystems.feeder;

import org.team9140.frc2026.Constants.Ports;
import org.team9140.frc2026.Constants.Feeder;
import org.team9140.lib.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOReal implements FeederIO{
    private final TalonFX feederMotor;
    private final VoltageOut voltageController;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public FeederIOReal() {
        feederMotor = new TalonFX(Ports.HOPPER_FEEDER_MOTOR, Ports.CANIVORE);
        voltageController = new VoltageOut(0);

        CurrentLimitsConfigs feederCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Feeder.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Feeder.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs feederMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        
        TalonFXConfiguration feederMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(feederCurrentLimits)
                .withMotorOutput(feederMotorOutputConfig);
        
        Util.tryUntilOk(() -> feederMotor.getConfigurator().apply(feederMotorConfigs));

        this.appliedVoltage = feederMotor.getMotorVoltage(false);
        this.supplyCurrent = feederMotor.getSupplyCurrent(false);
        this.torqueCurrent = feederMotor.getTorqueCurrent(false);
        this.tempCelsius = feederMotor.getDeviceTemp(false);

        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)); 
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(feederMotor));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        StatusCode spinnerMotorStatus =  BaseStatusSignal.refreshAll(
                                        appliedVoltage,
                                        supplyCurrent,
                                        torqueCurrent,
                                        tempCelsius);
        
        inputs.connected = connectedDebouncer.calculate(spinnerMotorStatus.isOK());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void runVoltage(double voltage) {
        feederMotor.setControl(voltageController.withOutput(voltage));
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
