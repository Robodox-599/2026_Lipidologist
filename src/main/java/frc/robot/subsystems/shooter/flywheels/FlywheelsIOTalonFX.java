package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
// import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelsIOTalonFX extends FlywheelsIO {
    // motors + configuration
    private final CANBus flywheelCANBus;

    private final TalonFX flywheelLeaderMotor;
    private final TalonFX flywheelFollower1Motor;
    private final TalonFX flywheelFollower2Motor;
    private final TalonFX flywheelFollower3Motor;

    TalonFXConfiguration flywheelConfiguration;
    private final VelocityVoltage velocityVoltage;
    // private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    // status signals
    private final StatusSignal<AngularVelocity> flywheelVelocityRPS;
    private final StatusSignal<Temperature> flywheelTemperature;
    private final StatusSignal<Angle> flywheelPosition;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelStatorCurrent;
    private final StatusSignal<Current> flywheelSupplyCurrent;

    // private final FlywheelConstants flywheelConstants;

    private Debouncer rpmDebouncer = new Debouncer(0.1, DebounceType.kBoth);
 
    public FlywheelsIOTalonFX() {
        // this.flywheelConstants = flywheelConstants;

        // motors + configuration
        flywheelCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);
        flywheelLeaderMotor = new TalonFX(FlywheelsConstants.flywheelLeaderMotorID, flywheelCANBus);
        flywheelFollower1Motor = new TalonFX(FlywheelsConstants.flywheelFollower1MotorID, flywheelCANBus);
        flywheelFollower2Motor = new TalonFX(FlywheelsConstants.flywheelFollower2MotorID, flywheelCANBus);
        flywheelFollower3Motor = new TalonFX(FlywheelsConstants.flywheelFollower3MotorID, flywheelCANBus);

        flywheelConfiguration = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withPeakReverseDutyCycle(0)
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
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
                                .withKP(FlywheelsConstants.flywheelRealkP)
                                .withKI(FlywheelsConstants.flywheelRealkI)
                                .withKD(FlywheelsConstants.flywheelRealkD)
                                .withKS(FlywheelsConstants.flywheelRealkS)
                                .withKV(FlywheelsConstants.flywheelRealkV));

       
        velocityVoltage = new VelocityVoltage(super.targetRPS);
        // velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);

        // Applying configuration
        PhoenixUtil.tryUntilOk(10, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> flywheelFollower1Motor.getConfigurator().apply(flywheelConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> flywheelFollower2Motor.getConfigurator().apply(flywheelConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> flywheelFollower3Motor.getConfigurator().apply(flywheelConfiguration, 1));

        flywheelFollower1Motor.setControl(new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Aligned));
        flywheelFollower2Motor.setControl(new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Aligned));
        flywheelFollower3Motor.setControl(new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Aligned));

        // status signals
        flywheelVelocityRPS = flywheelLeaderMotor.getVelocity();
        flywheelTemperature = flywheelLeaderMotor.getDeviceTemp();
        flywheelPosition = flywheelLeaderMotor.getPosition();
        flywheelAppliedVolts = flywheelLeaderMotor.getMotorVoltage();
        flywheelStatorCurrent = flywheelLeaderMotor.getStatorCurrent();
        flywheelSupplyCurrent = flywheelLeaderMotor.getSupplyCurrent();

        // update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocityRPS, flywheelTemperature,
                flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        flywheelLeaderMotor.optimizeBusUtilization();
        flywheelFollower1Motor.optimizeBusUtilization();
        flywheelFollower2Motor.optimizeBusUtilization();
        flywheelFollower3Motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(flywheelVelocityRPS, flywheelTemperature,
                flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        super.RPS = flywheelVelocityRPS.getValueAsDouble();
        super.statorCurrent = flywheelStatorCurrent.getValueAsDouble();
        super.supplyCurrent = flywheelSupplyCurrent.getValueAsDouble();
        super.temperature = flywheelTemperature.getValueAsDouble();
        super.isFlywheelAtSetpoint = rpmDebouncer.calculate(Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);

        DogLog.log("Flywheels/RPS", super.RPS);
        DogLog.log("Flywheels/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
        DogLog.log("Flywheels/isFlywheelAtSpeedRaw", Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);
        DogLog.log("Flywheels/statorCurrent", super.statorCurrent);
        DogLog.log("Flywheels/supplyCurrent", super.supplyCurrent);
        DogLog.log("Flywheels/temperature", super.temperature);
    }

    @Override
    public void setRPS(double RPS) {
        super.targetRPS = RPS;
        flywheelLeaderMotor.setControl(velocityVoltage.withVelocity(RPS).withEnableFOC(true));
        // flywheelMotor.setControl(velocityTorqueCurrentFOC.withVelocity(RPS));
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelLeaderMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        flywheelLeaderMotor.setVoltage(0);
    }
}
