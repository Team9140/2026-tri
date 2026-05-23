package org.team9140.frc2026.subsystems.turret;

import org.team9140.frc2026.Constants;
import org.team9140.lib.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOReal implements TurretIO{
    private final TalonFX turretMotor;
    private final CANcoder turretCANcoder;
    private final MotionMagicTorqueCurrentFOC MMControl;
    private final VoltageOut voltageControl;
    private final StaticBrake brake;

    private final StatusSignal<Angle> turretAngle;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final StatusSignal<Angle> CANcoderAbsoluteAngle;
    private final StatusSignal<Angle> CANcoderAngle;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public TurretIOReal() {
        turretMotor = new TalonFX(Constants.Ports.TURRET_MOTOR, Constants.Ports.CANIVORE);
        turretCANcoder = new CANcoder(Constants.Ports.TURRET_CANCODER, Constants.Ports.CANIVORE);
        MMControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        voltageControl = new VoltageOut(0);
        brake = new StaticBrake();
        
        // CONFIGS
        MotionMagicConfigs turretMotionMagicConfig = new MotionMagicConfigs()
                .withMotionMagicAcceleration(Constants.Turret.MM_ACCELERATION)
                .withMotionMagicCruiseVelocity(Constants.Turret.MM_CRUISE_VELOCITY);
        
        Slot0Configs turretSlot0Configs = new Slot0Configs()
                .withKS(Constants.Turret.KS)
                .withKV(Constants.Turret.KV)
                .withKA(Constants.Turret.KA)
                .withKP(Constants.Turret.KP)
                .withKI(Constants.Turret.KI)
                .withKD(Constants.Turret.KD);

        MotorOutputConfigs turretMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        
        TorqueCurrentConfigs turretTorqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Constants.Turret.STATOR_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(-Constants.Turret.STATOR_CURRENT_LIMIT);

        CurrentLimitsConfigs turretCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Turret.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Turret.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        SoftwareLimitSwitchConfigs turretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Turret.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Turret.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);

        FeedbackConfigs turretFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Turret.SENSOR_TO_MECHANISM)
                .withFeedbackRemoteSensorID(Constants.Ports.TURRET_CANCODER)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(Constants.Turret.GEAR_RATIO /
                        Constants.Turret.SENSOR_TO_MECHANISM);
        
        TalonFXConfiguration turretConfig = new TalonFXConfiguration()
                .withMotionMagic(turretMotionMagicConfig)
                .withSlot0(turretSlot0Configs)
                .withMotorOutput(turretMotorOutputConfigs)
                .withCurrentLimits(turretCurrentLimitsConfigs)
                .withTorqueCurrent(turretTorqueCurrentConfigs)
                .withSoftwareLimitSwitch(turretSoftwareLimitSwitchConfigs)
                .withFeedback(turretFeedbackConfigs);
        
        MagnetSensorConfigs turretCancoderConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Constants.Turret.CANCODER_OFFSET_ROTS);
        
        // Apply configs
        Util.tryUntilOk(() -> turretMotor.getConfigurator().apply(turretConfig));
        Util.tryUntilOk(() -> turretCANcoder.getConfigurator().apply(turretCancoderConfig));

        // fetch motor signals
        this.turretAngle = turretMotor.getPosition(false);
        this.appliedVoltage = turretMotor.getMotorVoltage(false);
        this.supplyCurrent = turretMotor.getSupplyCurrent(false);
        this.torqueCurrent = turretMotor.getTorqueCurrent(false);
        this.tempCelsius = turretMotor.getDeviceTemp(false);

        // fetch CANcoder signals
        this.CANcoderAbsoluteAngle = turretCANcoder.getAbsolutePosition(false);
        this.CANcoderAngle = turretCANcoder.getPosition(false);

        // set signal frequencies, may be changed to give some signals higher freqency
        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                turretAngle,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                CANcoderAbsoluteAngle,
                CANcoderAngle));
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretCANcoder));
        
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        StatusCode motorStatus = BaseStatusSignal.refreshAll(turretAngle,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius);
        
        StatusCode CANcoderStatus = BaseStatusSignal.refreshAll(
                CANcoderAbsoluteAngle,
                CANcoderAngle);
        
        inputs.connected = connectedDebouncer.calculate(motorStatus.isOK());
        inputs.turretAngleRotations = turretAngle.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        inputs.CANcoderConnected = connectedDebouncer.calculate(CANcoderStatus.isOK());
        inputs.CANcoderAbsolutePositionRot = CANcoderAbsoluteAngle.getValueAsDouble();
        inputs.CANcoderPositionRot = CANcoderAngle.getValueAsDouble();
    }

    @Override
    public void moveToPosition(double positionRot) {
        turretMotor.setControl(MMControl.withPosition(positionRot));
    }

    @Override
    public void runVoltage(double voltage) {
        turretMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void brake() {
        turretMotor.setControl(brake);
    }
}
