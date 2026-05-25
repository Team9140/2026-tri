package org.team9140.frc2026.subsystems.spinner;

import org.team9140.frc2026.Constants.Ports;
import org.team9140.frc2026.Constants.Spinner;
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

public class SpinnerIOReal implements SpinnerIO{
    protected final TalonFX spinnerMotor;
    private final VoltageOut voltageController;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public SpinnerIOReal() {
        spinnerMotor = new TalonFX(Ports.HOPPER_SPINNER_MOTOR, Ports.CANIVORE);
        voltageController = new VoltageOut(0);

        CurrentLimitsConfigs spinnerCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Spinner.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Spinner.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        MotorOutputConfigs spinnerMotorOutputConfig = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);
        
        TalonFXConfiguration spinnerMotorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(spinnerCurrentLimits)
                .withMotorOutput(spinnerMotorOutputConfig);
        
        Util.tryUntilOk(() -> spinnerMotor.getConfigurator().apply(spinnerMotorConfigs));

        this.appliedVoltage = spinnerMotor.getMotorVoltage(false);
        this.supplyCurrent = spinnerMotor.getSupplyCurrent(false);
        this.torqueCurrent = spinnerMotor.getTorqueCurrent(false);
        this.tempCelsius = spinnerMotor.getDeviceTemp(false);

        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)); 
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(spinnerMotor));
    }

    @Override
    public void updateInputs(SpinnerIOInputs inputs) {
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
        spinnerMotor.setControl(voltageController.withOutput(voltage));
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
