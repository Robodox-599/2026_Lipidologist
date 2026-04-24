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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
// import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelsIOTalonFX extends FlywheelsIO {
  // motors + configuration
  private final CANBus flywheelCANBus;

  private final TalonFX flywheelLeaderMotor;
  private final TalonFX flywheelBottomLeftMotor;
  private final TalonFX flywheelTopRightMotor;
  private final TalonFX flywheelBottomRightMotor;

  TalonFXConfiguration flywheelConfiguration;
  // private final VelocityVoltage velocityVoltage;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

  // status signals
  private final StatusSignal<AngularVelocity> flywheelLeaderVelocityRPS; // VelocityRPS
  private final StatusSignal<AngularVelocity> flywheelFollower1VelocityRPS;
  private final StatusSignal<AngularVelocity> flywheelFollower2VelocityRPS;
  private final StatusSignal<AngularVelocity> flywheelFollower3VelocityRPS;
  private final StatusSignal<Current> flywheelLeaderStatorCurrent; // StatorCurrent
  private final StatusSignal<Current> flywheelFollower1StatorCurrent;
  private final StatusSignal<Current> flywheelFollower2StatorCurrent;
  private final StatusSignal<Current> flywheelFollower3StatorCurrent;
  private final StatusSignal<Current> flywheelLeaderSupplyCurrent; // SupplyCurrent
  private final StatusSignal<Current> flywheelFollower1SupplyCurrent;
  private final StatusSignal<Current> flywheelFollower2SupplyCurrent;
  private final StatusSignal<Current> flywheelFollower3SupplyCurrent;
  private final StatusSignal<Voltage> flywheelLeaderAppliedVolts; // AppliedVolts
  private final StatusSignal<Voltage> flywheelFollower1AppliedVolts;
  private final StatusSignal<Voltage> flywheelFollower2AppliedVolts;
  private final StatusSignal<Voltage> flywheelFollower3AppliedVolts;
  private final BaseStatusSignal[] characterizationSignals;

  // private final FlywheelConstants flywheelConstants;

  private Debouncer rpmDebouncer = new Debouncer(0.1, DebounceType.kBoth);

  public FlywheelsIOTalonFX() {
    // this.flywheelConstants = flywheelConstants;

    // motors + configuration
    flywheelCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);
    flywheelLeaderMotor = new TalonFX(FlywheelsConstants.flywheelLeaderMotorID, flywheelCANBus);
    flywheelBottomLeftMotor =
        new TalonFX(FlywheelsConstants.flywheelBottomLeftMotorID, flywheelCANBus);
    flywheelTopRightMotor = new TalonFX(FlywheelsConstants.flywheelTopRightMotorID, flywheelCANBus);
    flywheelBottomRightMotor =
        new TalonFX(FlywheelsConstants.flywheelBottomRightMotorID, flywheelCANBus);

    flywheelConfiguration =
        new TalonFXConfiguration()
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
                    .withSupplyCurrentLimitEnable(false)
                    .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(FlywheelsConstants.flywheelRealkP)
                    .withKI(FlywheelsConstants.flywheelRealkI)
                    .withKD(FlywheelsConstants.flywheelRealkD)
                    .withKS(FlywheelsConstants.flywheelRealkS)
                    .withKV(FlywheelsConstants.flywheelRealkV));

    // velocityVoltage = new VelocityVoltage(super.targetRPS);
    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);

    // Applying configuration
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelLeaderMotor.getConfigurator().apply(flywheelConfiguration, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelBottomLeftMotor.getConfigurator().apply(flywheelConfiguration, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelTopRightMotor.getConfigurator().apply(flywheelConfiguration, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelBottomRightMotor.getConfigurator().apply(flywheelConfiguration, 1));

    flywheelBottomLeftMotor.setControl(
        new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Aligned));
    flywheelTopRightMotor.setControl(
        new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Opposed));
    flywheelBottomRightMotor.setControl(
        new Follower(FlywheelsConstants.flywheelLeaderMotorID, MotorAlignmentValue.Opposed));

    // status signals
    flywheelLeaderVelocityRPS = flywheelLeaderMotor.getVelocity(); // VelocityRPS
    flywheelFollower1VelocityRPS = flywheelBottomLeftMotor.getVelocity();
    flywheelFollower2VelocityRPS = flywheelTopRightMotor.getVelocity();
    flywheelFollower3VelocityRPS = flywheelBottomRightMotor.getVelocity();
    flywheelLeaderStatorCurrent = flywheelLeaderMotor.getStatorCurrent(); // StatorCurrent
    flywheelFollower1StatorCurrent = flywheelBottomLeftMotor.getStatorCurrent();
    flywheelFollower2StatorCurrent = flywheelTopRightMotor.getStatorCurrent();
    flywheelFollower3StatorCurrent = flywheelBottomRightMotor.getStatorCurrent();
    flywheelLeaderSupplyCurrent = flywheelLeaderMotor.getSupplyCurrent(); // SupplyCurrent
    flywheelFollower1SupplyCurrent = flywheelBottomLeftMotor.getSupplyCurrent();
    flywheelFollower2SupplyCurrent = flywheelTopRightMotor.getSupplyCurrent();
    flywheelFollower3SupplyCurrent = flywheelBottomRightMotor.getSupplyCurrent();
    flywheelLeaderAppliedVolts = flywheelLeaderMotor.getMotorVoltage(); //
    flywheelFollower1AppliedVolts = flywheelBottomLeftMotor.getMotorVoltage();
    flywheelFollower2AppliedVolts = flywheelTopRightMotor.getMotorVoltage();
    flywheelFollower3AppliedVolts = flywheelBottomRightMotor.getMotorVoltage();
    characterizationSignals =
        new BaseStatusSignal[] {
          flywheelLeaderVelocityRPS,
          flywheelFollower1VelocityRPS,
          flywheelFollower2VelocityRPS,
          flywheelFollower3VelocityRPS,
          flywheelLeaderStatorCurrent,
          flywheelFollower1StatorCurrent,
          flywheelFollower2StatorCurrent,
          flywheelFollower3StatorCurrent,
          flywheelLeaderSupplyCurrent,
          flywheelFollower1SupplyCurrent,
          flywheelFollower2SupplyCurrent,
          flywheelFollower2SupplyCurrent,
          flywheelFollower3SupplyCurrent,
          flywheelLeaderAppliedVolts,
          flywheelFollower1AppliedVolts,
          flywheelFollower2AppliedVolts,
          flywheelFollower3AppliedVolts
        };

    // update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(50, characterizationSignals);

    flywheelLeaderMotor.optimizeBusUtilization();
    flywheelBottomLeftMotor.optimizeBusUtilization();
    flywheelTopRightMotor.optimizeBusUtilization();
    flywheelBottomRightMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(characterizationSignals);

    super.RPS = flywheelLeaderVelocityRPS.getValueAsDouble();
    super.statorCurrent = flywheelLeaderStatorCurrent.getValueAsDouble();
    super.supplyCurrent = flywheelLeaderSupplyCurrent.getValueAsDouble();
    super.isFlywheelAtSetpoint =
        rpmDebouncer.calculate(
            Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);

    DogLog.log("Flywheels/RPS", super.RPS);
    DogLog.log("Flywheels/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
    DogLog.log(
        "Flywheels/isFlywheelAtSpeedRaw",
        Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);
    DogLog.log("Flywheels/statorCurrent", super.statorCurrent);
    DogLog.log("Flywheels/supplyCurrent", super.supplyCurrent);
    DogLog.log("Flywheels/temperature", super.temperature);
  }

  @Override
  public void setRPS(double RPS) {
    super.targetRPS = RPS;
    // flywheelLeaderMotor.setControl(velocityVoltage.withVelocity(RPS).withEnableFOC(true));
    flywheelLeaderMotor.setControl(velocityTorqueCurrentFOC.withVelocity(RPS));
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
