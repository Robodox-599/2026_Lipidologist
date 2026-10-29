package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakeRollersIOTalonFX extends IntakeRollersIO {
  public final TalonFX intakeRollersLeftMotor;
  public final TalonFX intakeRollersRightMotor;
  public final TalonFXConfiguration intakeRollersConfig;
  public final CANBus intakeRollersCanBus;

  private final DutyCycleOut dutyCycleRequest;
  //   private final DutyCycleOut followerDutyCycleRequest;

  // status signals
  private final StatusSignal<AngularVelocity> intakeRollersLeaderVelocity;
  private final StatusSignal<Voltage> intakeRollersLeaderAppliedVolts;
  private final StatusSignal<Current> intakeRollersLeaderSupplyCurrent;
  private final StatusSignal<Current> intakeRollersLeaderStatorCurrent;
  private final StatusSignal<Temperature> intakeRollersLeaderTemperature;

  private final StatusSignal<AngularVelocity> intakeRollersFollowerVelocity;
  private final StatusSignal<Voltage> intakeRollersFollowerAppliedVolts;
  private final StatusSignal<Current> intakeRollersFollowerSupplyCurrent;
  private final StatusSignal<Current> intakeRollersFollowerStatorCurrent;
  private final StatusSignal<Temperature> intakeRollersFollowerTemperature;

  public IntakeRollersIOTalonFX() {
    intakeRollersCanBus = new CANBus(IntakeRollersConstants.intakeRollersCanBus);
    intakeRollersLeftMotor =
        new TalonFX(IntakeRollersConstants.intakeRollersLeftMotorID, intakeRollersCanBus);
    intakeRollersRightMotor =
        new TalonFX(IntakeRollersConstants.intakeRollersRightMotorID, intakeRollersCanBus);

    intakeRollersConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
    // .withOpenLoopRamps(new
    // OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1.0))
    ;

    dutyCycleRequest = new DutyCycleOut(0);
    // followerDutyCycleRequest = new DutyCycleOut(0);

    intakeRollersLeaderVelocity = intakeRollersLeftMotor.getVelocity();
    intakeRollersLeaderAppliedVolts = intakeRollersLeftMotor.getMotorVoltage();
    intakeRollersLeaderSupplyCurrent = intakeRollersLeftMotor.getSupplyCurrent();
    intakeRollersLeaderStatorCurrent = intakeRollersLeftMotor.getStatorCurrent();
    intakeRollersLeaderTemperature = intakeRollersLeftMotor.getDeviceTemp();

    intakeRollersFollowerVelocity = intakeRollersRightMotor.getVelocity();
    intakeRollersFollowerAppliedVolts = intakeRollersRightMotor.getMotorVoltage();
    intakeRollersFollowerSupplyCurrent = intakeRollersRightMotor.getSupplyCurrent();
    intakeRollersFollowerStatorCurrent = intakeRollersRightMotor.getStatorCurrent();
    intakeRollersFollowerTemperature = intakeRollersRightMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        // intake rollers 1
        intakeRollersLeaderVelocity,
        intakeRollersLeaderStatorCurrent,
        intakeRollersLeaderSupplyCurrent,
        // intake rollers 2
        intakeRollersFollowerVelocity,
        intakeRollersFollowerStatorCurrent,
        intakeRollersFollowerSupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        5,
        // intake rollers 1
        intakeRollersLeaderTemperature,
        intakeRollersLeaderAppliedVolts,
        // intake rollers 2
        intakeRollersFollowerTemperature,
        intakeRollersFollowerAppliedVolts);

    PhoenixUtil.tryUntilOk(
        10, () -> intakeRollersLeftMotor.getConfigurator().apply(intakeRollersConfig, 1));
    PhoenixUtil.tryUntilOk(
        10, () -> intakeRollersRightMotor.getConfigurator().apply(intakeRollersConfig, 1));

    intakeRollersLeftMotor.optimizeBusUtilization();
    intakeRollersRightMotor.optimizeBusUtilization();
  }

  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        // intake rollers 1
        intakeRollersLeaderVelocity,
        intakeRollersLeaderStatorCurrent,
        intakeRollersLeaderSupplyCurrent,
        intakeRollersLeaderAppliedVolts,
        intakeRollersLeaderTemperature,
        // intake rollers 2
        intakeRollersFollowerVelocity,
        intakeRollersFollowerStatorCurrent,
        intakeRollersFollowerSupplyCurrent,
        intakeRollersFollowerAppliedVolts,
        intakeRollersFollowerTemperature);

    // intake rollers 1
    super.intakeRollersLeaderVelocity = intakeRollersLeaderVelocity.getValueAsDouble();
    super.intakeRollersLeaderVoltage = intakeRollersLeaderAppliedVolts.getValueAsDouble();
    super.intakeRollersLeaderStatorCurrent = intakeRollersLeaderStatorCurrent.getValueAsDouble();
    super.intakeRollersLeaderSupplyCurrent = intakeRollersLeaderSupplyCurrent.getValueAsDouble();
    super.intakeRollersLeaderTemperature = intakeRollersLeaderTemperature.getValueAsDouble();

    DogLog.log("Intake/Rollers/Left/Velocity", super.intakeRollersLeaderVelocity);
    DogLog.log("Intake/Rollers/Left/Voltage", super.intakeRollersLeaderVoltage);
    DogLog.log("Intake/Rollers/Left/StatorCurrent", super.intakeRollersLeaderStatorCurrent);
    DogLog.log("Intake/Rollers/Left/SupplyCurrent", super.intakeRollersLeaderSupplyCurrent);
    DogLog.log("Intake/Rollers/Left/Temperature", super.intakeRollersLeaderTemperature);
    // intake rollers 2
    super.intakeRollersFollowerVelocity = intakeRollersFollowerVelocity.getValueAsDouble();
    super.intakeRollersFollowerVoltage = intakeRollersFollowerAppliedVolts.getValueAsDouble();
    super.intakeRollersFollowerStatorCurrent =
        intakeRollersFollowerStatorCurrent.getValueAsDouble();
    super.intakeRollersFollowerSupplyCurrent =
        intakeRollersFollowerSupplyCurrent.getValueAsDouble();
    super.intakeRollersFollowerTemperature = intakeRollersFollowerTemperature.getValueAsDouble();

    DogLog.log("Intake/Rollers/Right/Velocity", super.intakeRollersFollowerVelocity);
    DogLog.log("Intake/Rollers/Right/Voltage", super.intakeRollersFollowerVoltage);
    DogLog.log("Intake/Rollers/Right/StatorCurrent", super.intakeRollersFollowerStatorCurrent);
    DogLog.log("Intake/Rollers/Right/SupplyCurrent", super.intakeRollersFollowerSupplyCurrent);
    DogLog.log("Intake/Rollers/Right/Temperature", super.intakeRollersFollowerTemperature);
  }

  @Override
  public void stopIntakeRollers() {
    intakeRollersLeftMotor.setVoltage(0);
    intakeRollersRightMotor.setVoltage(0);
  }

  @Override
  public void setIntakeRollersDutyCycle(double dutyCycle) {
    intakeRollersLeftMotor.setControl(dutyCycleRequest.withOutput(dutyCycle).withEnableFOC(true));
    intakeRollersRightMotor.setControl(dutyCycleRequest.withOutput(dutyCycle).withEnableFOC(true));
  }
}
