package org.team9140.frc2026.subsystems.shooter;

import org.team9140.frc2026.Constants.Ports;
import org.team9140.frc2026.Constants.Shooter;
import org.team9140.lib.Util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO{
    private final TalonFX shooterMotor;
    private final TalonFX shooterFollower;
    private final VelocityTorqueCurrentFOC shooterController;
    private final CoastOut shooterCoastOut;

    private final StatusSignal<AngularVelocity> shooterVelocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final StatusSignal<Voltage> followerAppliedVoltage;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerTempCelsius;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    public ShooterIOReal() {
        shooterMotor = new TalonFX(Ports.SHOOTER_MOTOR, Ports.SHOOTER_CANIVORE);
        shooterFollower = new TalonFX(Ports.SHOOTER_FOLLOWER_MOTOR, Ports.SHOOTER_CANIVORE);

        shooterController = new VelocityTorqueCurrentFOC(0).withSlot(0);
        shooterCoastOut = new CoastOut();

        Slot0Configs shooterSlot0Configs = new Slot0Configs()
                .withKS(Shooter.KS)
                .withKV(Shooter.KV)
                .withKA(Shooter.KA)
                .withKP(Shooter.KP)
                .withKI(Shooter.KI)
                .withKD(Shooter.KD);
        
        MotorOutputConfigs shooterMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        
        TorqueCurrentConfigs shooterTorqueCurrentConfigs = new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(Shooter.PEAK_FORWARD_TORQUE)
                .withPeakReverseTorqueCurrent(Shooter.PEAK_REVERSE_TORQUE);

        CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Shooter.PEAK_FORWARD_TORQUE)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Shooter.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        
        FeedbackConfigs shooterFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Shooter.GEAR_RATIO);
        
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(shooterSlot0Configs)
                .withMotorOutput(shooterMotorOutputConfigs)
                .withTorqueCurrent(shooterTorqueCurrentConfigs)
                .withCurrentLimits(shooterCurrentLimits)
                .withFeedback(shooterFeedbackConfigs);
        
        Util.tryUntilOk(() -> shooterMotor.getConfigurator().apply(shooterConfig));
        Util.tryUntilOk(() -> shooterFollower.getConfigurator().apply(shooterConfig));
        Util.tryUntilOk(() -> shooterFollower.setControl(new Follower(Ports.SHOOTER_MOTOR, MotorAlignmentValue.Opposed)));
        
        this.shooterVelocity = shooterMotor.getVelocity(false);
        this.appliedVoltage = shooterMotor.getMotorVoltage(false);
        this.supplyCurrent = shooterMotor.getSupplyCurrent(false);
        this.torqueCurrent = shooterMotor.getTorqueCurrent(false);
        this.tempCelsius = shooterMotor.getDeviceTemp(false);

        this.followerAppliedVoltage = shooterFollower.getMotorVoltage(false);
        this.followerSupplyCurrent = shooterFollower.getSupplyCurrent(false);
        this.followerTorqueCurrent = shooterFollower.getTorqueCurrent(false);
        this.followerTempCelsius = shooterFollower.getDeviceTemp(false);

        Util.tryUntilOk(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
                shooterVelocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerTempCelsius));
        
        Util.tryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(shooterMotor, shooterFollower)); 
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        StatusCode shooterMotorStatus =  BaseStatusSignal.refreshAll(
                                        shooterVelocity,
                                        appliedVoltage,
                                        supplyCurrent,
                                        torqueCurrent,
                                        tempCelsius);
        
        StatusCode shooterFollowerStatus = BaseStatusSignal.refreshAll(
                                            followerAppliedVoltage,
                                            followerSupplyCurrent,
                                            followerTorqueCurrent,
                                            followerTempCelsius);

        // update main motor inputs
        inputs.connected = connectedDebouncer.calculate(shooterMotorStatus.isOK());
        inputs.velocityRotsPerSec = shooterVelocity.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        // update follower motor inputs
        inputs.followerConnected = connectedDebouncer.calculate(shooterFollowerStatus.isOK());
        inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
    }

    @Override
    public void goToVelocity(double velocityRotsPerSec) {
        shooterMotor.setControl(shooterController.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void off() {
        shooterMotor.setControl(shooterCoastOut);
    }
}
