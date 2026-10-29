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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import edu.wpi.first.math.filter.Debouncer;

public class FlywheelsIOTalonFX extends FlywheelsIO {
  private final TalonFX flywheelTopLeftMotor;
  private final TalonFX flywheelBottomLeftMotor;
  private final TalonFX flywheelTopRightMotor;
  private final TalonFX flywheelBottomRightMotor;
  private final CANBus flywheelCANBus;
  private final TalonFXConfiguration flywheelConfig;
  private final Debouncer entryDebouncer;
  private final Debouncer exitDebouncer;

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut voltageOut;

  // status signals
  private final StatusSignal<AngularVelocity> flywheelTopLeftVelocityRPS;
  private final StatusSignal<AngularVelocity> flywheelBottomLeftVelocityRPS;
  private final StatusSignal<AngularVelocity> flywheelTopRightVelocityRPS;
  private final StatusSignal<AngularVelocity> flywheelBottomRightVelocityRPS;

  private final StatusSignal<Current> flywheelTopLeftStatorCurrent;
  private final StatusSignal<Current> flywheelBottomLeftStatorCurrent;
  private final StatusSignal<Current> flywheelTopRightStatorCurrent;
  private final StatusSignal<Current> flywheelBottomRightStatorCurrent;

  private final StatusSignal<Current> flywheelTopLeftSupplyCurrent;
  private final StatusSignal<Current> flywheelBottomLeftSupplyCurrent;
  private final StatusSignal<Current> flywheelTopRightSupplyCurrent;
  private final StatusSignal<Current> flywheelBottomRightSupplyCurrent;

  private final StatusSignal<Voltage> flywheelTopLeftAppliedVolts;
  private final StatusSignal<Voltage> flywheelBottomLeftAppliedVolts;
  private final StatusSignal<Voltage> flywheelTopRightAppliedVolts;
  private final StatusSignal<Voltage> flywheelBottomRightAppliedVolts;
  private final StatusSignal<Temperature> flywheelTopLeftTemperature;
  
  private final BaseStatusSignal[] criticalSignals;
  private final BaseStatusSignal[] lowPrioritySignals;

  public FlywheelsIOTalonFX() {
    // motor stuff
    flywheelCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);
    flywheelTopLeftMotor = new TalonFX(FlywheelsConstants.flywheelTopLeftMotorID, flywheelCANBus);
    flywheelBottomLeftMotor =
        new TalonFX(FlywheelsConstants.flywheelBottomLeftMotorID, flywheelCANBus);
    flywheelTopRightMotor = new TalonFX(FlywheelsConstants.flywheelTopRightMotorID, flywheelCANBus);
    flywheelBottomRightMotor =
        new TalonFX(FlywheelsConstants.flywheelBottomRightMotorID, flywheelCANBus);

    flywheelConfig =
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

    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);
    voltageOut = new VoltageOut(0.0);

    entryDebouncer = new Debouncer(FlywheelsConstants.entryDebounceTime, Debouncer.DebounceType.kRising);
    exitDebouncer = new Debouncer(FlywheelsConstants.exitDebounceTime, Debouncer.DebounceType.kFalling);

    // status signal stuff
    flywheelTopLeftVelocityRPS = flywheelTopLeftMotor.getVelocity();
    flywheelBottomLeftVelocityRPS = flywheelBottomLeftMotor.getVelocity();
    flywheelTopRightVelocityRPS = flywheelTopRightMotor.getVelocity();
    flywheelBottomRightVelocityRPS = flywheelBottomRightMotor.getVelocity();

    flywheelTopLeftStatorCurrent = flywheelTopLeftMotor.getStatorCurrent();
    flywheelBottomLeftStatorCurrent = flywheelBottomLeftMotor.getStatorCurrent();
    flywheelTopRightStatorCurrent = flywheelTopRightMotor.getStatorCurrent();
    flywheelBottomRightStatorCurrent = flywheelBottomRightMotor.getStatorCurrent();

    flywheelTopLeftSupplyCurrent = flywheelTopLeftMotor.getSupplyCurrent();
    flywheelBottomLeftSupplyCurrent = flywheelBottomLeftMotor.getSupplyCurrent();
    flywheelTopRightSupplyCurrent = flywheelTopRightMotor.getSupplyCurrent();
    flywheelBottomRightSupplyCurrent = flywheelBottomRightMotor.getSupplyCurrent();

    flywheelTopLeftAppliedVolts = flywheelTopLeftMotor.getMotorVoltage();
    flywheelBottomLeftAppliedVolts = flywheelBottomLeftMotor.getMotorVoltage();
    flywheelTopRightAppliedVolts = flywheelTopRightMotor.getMotorVoltage();
    flywheelBottomRightAppliedVolts = flywheelBottomRightMotor.getMotorVoltage();

    flywheelTopLeftTemperature = flywheelTopLeftMotor.getDeviceTemp();

    criticalSignals =
        new BaseStatusSignal[] {
          flywheelTopLeftVelocityRPS,
          flywheelTopLeftStatorCurrent,
          flywheelTopLeftSupplyCurrent
        };

    lowPrioritySignals =
        new BaseStatusSignal[] {
          flywheelBottomLeftVelocityRPS,
          flywheelTopRightVelocityRPS,
          flywheelBottomRightVelocityRPS,
          flywheelBottomLeftStatorCurrent,
          flywheelTopRightStatorCurrent,
          flywheelBottomRightStatorCurrent,
          flywheelBottomLeftSupplyCurrent,
          flywheelTopRightSupplyCurrent,
          flywheelBottomRightSupplyCurrent,
          flywheelTopLeftAppliedVolts,
          flywheelBottomLeftAppliedVolts,
          flywheelTopRightAppliedVolts,
          flywheelBottomRightAppliedVolts,
          flywheelTopLeftTemperature
        };


    BaseStatusSignal.setUpdateFrequencyForAll(50, criticalSignals);

    BaseStatusSignal.setUpdateFrequencyForAll(5, lowPrioritySignals);

    PhoenixUtil.tryUntilOk(
        10, () -> flywheelBottomLeftMotor.getConfigurator().apply(flywheelConfig, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelBottomRightMotor.getConfigurator().apply(flywheelConfig, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelTopLeftMotor.getConfigurator().apply(flywheelConfig, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> flywheelTopRightMotor.getConfigurator().apply(flywheelConfig, 1));

    flywheelBottomLeftMotor.setControl(
        new Follower(flywheelTopLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    flywheelTopRightMotor.setControl(
        new Follower(flywheelTopLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelBottomRightMotor.setControl(
        new Follower(flywheelTopLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    flywheelTopLeftMotor.optimizeBusUtilization();
    flywheelBottomLeftMotor.optimizeBusUtilization();
    flywheelTopRightMotor.optimizeBusUtilization();
    flywheelBottomRightMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(criticalSignals);
    BaseStatusSignal.refreshAll(lowPrioritySignals);

    super.RPS = flywheelTopLeftVelocityRPS.getValueAsDouble();
    super.statorCurrent = flywheelTopLeftStatorCurrent.getValueAsDouble();
    super.supplyCurrent = flywheelTopLeftSupplyCurrent.getValueAsDouble();
    super.temperature = flywheelTopLeftTemperature.getValueAsDouble();

    super.withinExitTolerance = Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSExitTolerance;
    super.withinEntryTolerance = Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSEntryTolerance;

    if (super.isFlywheelAtSetpoint) {
      super.isFlywheelAtSetpoint = exitDebouncer.calculate(withinExitTolerance);
    } else {
      super.isFlywheelAtSetpoint = entryDebouncer.calculate(withinEntryTolerance);
    }


    DogLog.log("Flywheel/TopLeft/RPS", super.RPS);
    DogLog.log("Flywheel/BottomLeft/RPS", flywheelBottomLeftVelocityRPS.getValueAsDouble());
    DogLog.log("Flywheel/TopRight/RPS", flywheelTopRightVelocityRPS.getValueAsDouble());
    DogLog.log("Flywheel/BottomRight/RPS", flywheelBottomRightVelocityRPS.getValueAsDouble());
   
    DogLog.log("Flywheel/TopLeft/statorCurrent", super.statorCurrent);
    DogLog.log("Flywheel/BottomLeft/statorCurrent", flywheelBottomLeftStatorCurrent.getValueAsDouble());
    DogLog.log("Flywheel/TopRight/statorCurrent", flywheelTopRightStatorCurrent.getValueAsDouble());
    DogLog.log("Flywheel/BottomRight/statorCurrent", flywheelBottomRightStatorCurrent.getValueAsDouble());

    DogLog.log("Flywheel/TopLeft/supplyCurrent", super.supplyCurrent);
    DogLog.log("Flywheel/BottomLeft/supplyCurrent", flywheelBottomLeftSupplyCurrent.getValueAsDouble());
    DogLog.log("Flywheel/TopRight/supplyCurrent", flywheelTopRightSupplyCurrent.getValueAsDouble());
    DogLog.log("Flywheel/BottomRight/supplyCurrent", flywheelBottomRightSupplyCurrent.getValueAsDouble());

    DogLog.log("Flywheel/TopLeft/temperature", super.temperature);

    DogLog.log("Flywheel/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
  }

  @Override
  public void setFlywheelsRPS(double RPS) {
    super.targetRPS = RPS;
    flywheelTopLeftMotor.setControl(velocityTorqueCurrentFOC.withVelocity(RPS));
  }

  @Override
  public void setFlywheelsDutyCycle(double dutyCycle) {
    flywheelTopLeftMotor.setControl(dutyCycleRequest.withOutput(dutyCycle).withEnableFOC(true));
  }

  @Override
  public void stopFlywheels() {
    flywheelTopLeftMotor.setControl(voltageOut.withOutput(0).withEnableFOC(true));
  }

  @Override
  public void setFlywheelVoltage(double voltage) {
    flywheelTopLeftMotor.setControl(voltageOut.withOutput(voltage).withEnableFOC(true));
  }
}
