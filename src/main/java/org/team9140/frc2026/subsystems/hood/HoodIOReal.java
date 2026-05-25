package org.team9140.frc2026.subsystems.hood;

import org.team9140.frc2026.Constants.Hood;
import org.team9140.frc2026.Constants.Ports;
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

public class HoodIOReal implements HoodIO{
    protected final TalonFX hoodMotor;
    protected final CANcoder hoodCANcoder;
    private final MotionMagicTorqueCurrentFOC MMcontroller;

    private final StatusSignal<Angle> hoodAngle;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final StatusSignal<Angle> CANcoderAbsoluteAngle;
    private final StatusSignal<Angle> CANcoderAngle;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public HoodIOReal() {
        hoodMotor = new TalonFX(Ports.HOOD_MOTOR, Ports.SHOOTER_CANIVORE);
        hoodCANcoder = new CANcoder(Ports.HOOD_CANCODER, Ports.SHOOTER_CANIVORE);

        MMcontroller = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

        MotionMagicConfigs hoodMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Hood.MM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Hood.MM_ACCELERATION);
        
        Slot0Configs hoodSlot0Configs = new Slot0Configs()
                .withKS(Hood.KS)
                .withKV(Hood.KV)
                .withKA(Hood.KA)
                .withKP(Hood.KP)
                .withKI(Hood.KI)
                .withKD(Hood.KD);
        
        MotorOutputConfigs hoodMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        
        TorqueCurrentConfigs hoodTorqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Hood.STATOR_CURRENT_LIMIT)
                .withPeakReverseTorqueCurrent(-Hood.STATOR_CURRENT_LIMIT);
        
        CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Hood.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Hood.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        SoftwareLimitSwitchConfigs hoodSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Hood.FORWARD_SOFT_LIMIT_THRESHOLD)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Hood.REVERSE_SOFT_LIMIT_THRESHOLD)
                .withReverseSoftLimitEnable(true);
        
        FeedbackConfigs hoodFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Hood.SENSOR_TO_MECHANISM_RATIO)
                .withRotorToSensorRatio(
                        Hood.GEAR_RATIO / Hood.SENSOR_TO_MECHANISM_RATIO)
                .withFeedbackRemoteSensorID(Ports.HOOD_CANCODER)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration()
                .withMotionMagic(hoodMotionMagicConfig)
                .withSlot0(hoodSlot0Configs)
                .withMotorOutput(hoodMotorOutputConfigs)
                .withCurrentLimits(hoodCurrentLimitsConfigs)
                .withTorqueCurrent(hoodTorqueCurrentConfigs)
                .withSoftwareLimitSwitch(hoodSoftwareLimitSwitchConfigs)
                .withFeedback(hoodFeedbackConfigs);
        
        MagnetSensorConfigs hoodCANcoderConfig = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Hood.CANCODER_OFFSET_ROTS);
        
        // apply configs
        Util.tryUntilOk(() -> hoodMotor.getConfigurator().apply(hoodMotorConfig));
        Util.tryUntilOk(() -> hoodCANcoder.getConfigurator().apply(hoodCANcoderConfig));

        // fetch motor signals
        this.hoodAngle = hoodMotor.getPosition(false);
        this.appliedVoltage = hoodMotor.getMotorVoltage(false);
        this.supplyCurrent = hoodMotor.getSupplyCurrent(false);
        this.torqueCurrent = hoodMotor.getTorqueCurrent(false);
        this.tempCelsius = hoodMotor.getDeviceTemp(false);

        // fetch CANcoder signals
        this.CANcoderAbsoluteAngle = hoodCANcoder.getAbsolutePosition(false);
        this.CANcoderAngle = hoodCANcoder.getPosition(false);

        // set signal frequencys, consider setting some like CANcoder to higher freq
        // could also set all to 100 since that is default but idk, this is a landwehr question
        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                hoodAngle,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                CANcoderAbsoluteAngle,
                CANcoderAngle)); 
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(hoodMotor, hoodCANcoder));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        StatusCode motorStatus = BaseStatusSignal.refreshAll(hoodAngle,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius);
        
        StatusCode CANcoderStatus = BaseStatusSignal.refreshAll(
                CANcoderAbsoluteAngle,
                CANcoderAngle);
        
        inputs.connected = connectedDebouncer.calculate(motorStatus.isOK());
        inputs.hoodAngleRotations = hoodAngle.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        inputs.CANcoderConnected = connectedDebouncer.calculate(CANcoderStatus.isOK());
        inputs.CANcoderAbsolutePositionRot = CANcoderAbsoluteAngle.getValueAsDouble();
        inputs.CANcoderPositionRot = CANcoderAngle.getValueAsDouble();
    }

    @Override
    public void moveToPosition(double angleRotations) {
        hoodMotor.setControl(MMcontroller.withPosition(angleRotations));
    }
}
