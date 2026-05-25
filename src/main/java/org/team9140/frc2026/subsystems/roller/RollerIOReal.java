package org.team9140.frc2026.subsystems.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import org.team9140.frc2026.Constants.Ports;
import org.team9140.frc2026.Constants.Roller;
import org.team9140.lib.Util;

public class RollerIOReal implements RollerIO{
    protected final TalonFX rollerMotor;
    protected final TalonFX rollerFollowerMotor;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final StatusSignal<Voltage> followerAppliedVoltage;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerTempCelsius;

    // Debounces connection input so random loss of connection from smth for short amount of time isn't logged
    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public RollerIOReal() {
        rollerMotor = new TalonFX(Ports.ROLLER_MOTOR, Ports.ROBO_RIO);
        rollerFollowerMotor = new TalonFX(Ports.ROLLER_FOLLOWER_MOTOR, Ports.ROBO_RIO);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Roller.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Roller.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(outputConfigs);
        
        Util.tryUntilOk(() -> this.rollerMotor.getConfigurator().apply(configs));
        Util.tryUntilOk(() -> this.rollerFollowerMotor.getConfigurator().apply(configs));
        Util.tryUntilOk(() -> this.rollerFollowerMotor.setControl(
                new Follower(Ports.ROLLER_MOTOR, MotorAlignmentValue.Opposed)));

        // fetch main motor's signals
        this.appliedVoltage = rollerMotor.getMotorVoltage(false);
        this.supplyCurrent = rollerMotor.getSupplyCurrent(false);
        this.torqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.tempCelsius = rollerMotor.getDeviceTemp(false);

        // fetch follower motor's signals
        this.followerAppliedVoltage = rollerFollowerMotor.getMotorVoltage(false);
        this.followerSupplyCurrent = rollerFollowerMotor.getSupplyCurrent(false);
        this.followerTorqueCurrent = rollerFollowerMotor.getTorqueCurrent(false);
        this.followerTempCelsius = rollerFollowerMotor.getDeviceTemp(false);

        // Set frequency for signals being used
        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerTempCelsius));
        
        // Reduces frequency of all unused signals to reduce CANBus utilization
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(rollerMotor, rollerFollowerMotor));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        StatusCode rollerMotorStatus =  BaseStatusSignal.refreshAll(
                                        appliedVoltage,
                                        supplyCurrent,
                                        torqueCurrent,
                                        tempCelsius);
        
        StatusCode rollerFollowerMotorStatus = BaseStatusSignal.refreshAll(
                                            followerAppliedVoltage,
                                            followerSupplyCurrent,
                                            followerTorqueCurrent,
                                            followerTempCelsius);
        
        // update main motor inputs
        inputs.connected = connectedDebouncer.calculate(rollerMotorStatus.isOK());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        // update follower motor inputs
        inputs.followerConnected = connectedDebouncer.calculate(rollerFollowerMotorStatus.isOK());
        inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
    } 

    @Override
    public void runVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void off() {
        runVoltage(0);
    }
}
