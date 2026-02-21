package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelsIOTalonFX extends FlywheelsIO {
    // motors + configuration
    private final CANBus flywheelCANBus;
    private final TalonFX flywheelMotor;
    TalonFXConfiguration flywheelConfiguration;

    // status signals
    private final StatusSignal<AngularVelocity> flywheelVelocityRPS;
    private final StatusSignal<Temperature> flywheelTemperature;
    private final StatusSignal<Angle> flywheelPosition;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelStatorCurrent;
    private final StatusSignal<Current> flywheelSupplyCurrent;

    private final FlywheelConstants flywheelConstants;

    public FlywheelsIOTalonFX(FlywheelConstants flywheelConstants) {
        this.flywheelConstants = flywheelConstants;

        // motors + configuration
        flywheelCANBus = new CANBus(this.flywheelConstants.CANBus());
        flywheelMotor = new TalonFX(this.flywheelConstants.motorID(), flywheelCANBus);
        flywheelConfiguration = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withPeakReverseDutyCycle(0))
                .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(0))
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(FlywheelsConstants.supplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
                                .withStatorCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(this.flywheelConstants.kP())
                                .withKI(this.flywheelConstants.kI())
                                .withKD(this.flywheelConstants.kD())
                                .withKS(this.flywheelConstants.kS())
                                .withKV(this.flywheelConstants.kV()))
                .withMotorOutput(new MotorOutputConfigs().withInverted(this.flywheelConstants.invert())
                        .withNeutralMode(NeutralModeValue.Coast));

        // Applying configuration
        PhoenixUtil.tryUntilOk(10, () -> flywheelMotor.getConfigurator().apply(flywheelConfiguration, 1));

        // status signals
        flywheelVelocityRPS = flywheelMotor.getVelocity();
        flywheelTemperature = flywheelMotor.getDeviceTemp();
        flywheelPosition = flywheelMotor.getPosition();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelStatorCurrent = flywheelMotor.getStatorCurrent();
        flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();

        // update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocityRPS, flywheelTemperature,
                flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        flywheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(flywheelVelocityRPS, flywheelTemperature,
                flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        super.RPS = flywheelVelocityRPS.getValueAsDouble();
        super.statorCurrent = flywheelStatorCurrent.getValueAsDouble();
        super.supplyCurrent = flywheelSupplyCurrent.getValueAsDouble();
        super.temperature = flywheelTemperature.getValueAsDouble();
        super.isFlywheelAtSetpoint = Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance;

        DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/RPS", super.RPS);
        DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
        DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/statorCurrent", super.statorCurrent);
        DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/supplyCurrent", super.supplyCurrent);
        DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/temperature", super.temperature);
    }

    @Override
    public void setRPS(double RPS) {
        super.targetRPS = RPS;
        flywheelMotor.setControl(new VelocityVoltage(super.targetRPS));
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        flywheelMotor.setVoltage(0);
    }
}
