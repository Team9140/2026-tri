package org.team9140.frc2026.subsystems.extender;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import org.team9140.frc2026.Constants;
import org.team9140.frc2026.Constants.Ports;
import org.team9140.lib.Util;

public class ExtenderIOReal implements ExtenderIO{
    private final TalonFX extenderMotor;
    private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicConfigs motionMagicConfigs;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    // Debounces connection input so random loss of connection from smth for short amount of time isn't logged
    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public ExtenderIOReal() {
        extenderMotor = new TalonFX(Ports.EXTENDER_MOTOR, Ports.CANIVORE);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Extender.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Extender.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        this.motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.Extender.ARM_OUT_VELOCITY)
                .withMotionMagicAcceleration(Constants.Extender.MM_ACCELERATION);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Extender.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Extender.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Constants.Extender.STATOR_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(-Constants.Extender.STATOR_CURRENT_LIMIT)
                .withTorqueNeutralDeadband(1.0);

         TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(currentLimits)
                .withMotorOutput(motorOutputConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withSlot0(new Slot0Configs().withKP(Constants.Extender.KP))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.Extender.GEAR_RATIO))
                .withTorqueCurrent(torqueCurrentConfigs);
        
        this.extenderMotor.getConfigurator().apply(motorConfigs);

        // fetch main motor's signals
        this.motorPosition = this.extenderMotor.getPosition(false);
        this.appliedVoltage = this.extenderMotor.getMotorVoltage(false);
        this.supplyCurrent = this.extenderMotor.getSupplyCurrent(false);
        this.torqueCurrent = this.extenderMotor.getTorqueCurrent(false);
        this.tempCelsius = this.extenderMotor.getDeviceTemp(false);

        // Set frequency for signals being used
        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                this.motorPosition,
                this.appliedVoltage,
                this.supplyCurrent,
                this.torqueCurrent,
                this.tempCelsius
                ));
        
        // Reduces frequency of all unused signals to reduce CANBus utilization
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(this.extenderMotor));
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        StatusCode extenderMotorStatus =  BaseStatusSignal.refreshAll(
                                        this.motorPosition,
                                        this.appliedVoltage,
                                        this.supplyCurrent,
                                        this.torqueCurrent,
                                        this.tempCelsius);
        
        inputs.connected = connectedDebouncer.calculate(extenderMotorStatus.isOK());
        inputs.motorPosition = this.motorPosition.getValueAsDouble();
        inputs.intakePosition = this.getPosition();
        inputs.appliedVoltage = this.appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = this.supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = this.torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = this.tempCelsius.getValueAsDouble();
    }

    @Override
    public void goToPosition(double position, double maxVelocity) {
        this.motionMagicConfigs.withMotionMagicCruiseVelocity(maxVelocity);
        this.extenderMotor.getConfigurator().apply(this.motionMagicConfigs);
        this.extenderMotor.setControl(this.motionMagic.withPosition(position / Constants.Extender.PINION_CIRCUMFERENCE));
    }

    @Override
    public double getPosition() {
        return this.motorPosition.getValueAsDouble() * Constants.Extender.PINION_CIRCUMFERENCE;
    }
}
