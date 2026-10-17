package frc.robot.subsystems.intake.intakeWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakeWristIOTalonFX extends IntakeWristIO {
  public final TalonFX intakeWristMotor;
  public final CANBus intakeWristCanBus;
  public final TalonFXConfiguration intakeWristConfig;
  private MotionMagicVoltage m_request;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // status signal
  public final StatusSignal<Angle> intakeWristPosition;
  public final StatusSignal<AngularVelocity> intakeWristVelocity;
  public final StatusSignal<Voltage> intakeWristAppliedVolts;
  public final StatusSignal<Current> intakeWristStatorCurrent;
  public final StatusSignal<Current> intakeWristSupplyCurrent;
  public final StatusSignal<Temperature> intakeWristTemperature;
  private Debouncer wristStallDebouncer =
      new Debouncer(IntakeWristConstants.debounceTime, DebounceType.kBoth);

  public IntakeWristIOTalonFX() {
    intakeWristCanBus = new CANBus(IntakeWristConstants.intakeWristCanBus);
    intakeWristMotor = new TalonFX(IntakeWristConstants.intakeWristMotorID, intakeWristCanBus);
    m_request = new MotionMagicVoltage(0);

    intakeWristConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(IntakeWristConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeWristConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeWristConstants.kP)
                    .withKI(IntakeWristConstants.kI)
                    .withKD(IntakeWristConstants.kD)
                    .withKV(IntakeWristConstants.kV)
                    .withKS(IntakeWristConstants.kS)
                    .withKG(IntakeWristConstants.kG)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(IntakeWristConstants.maxVelocity)
                    .withMotionMagicAcceleration(IntakeWristConstants.maxAcceleration))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IntakeWristConstants.gearRatio));

    intakeWristPosition = intakeWristMotor.getPosition();
    intakeWristVelocity = intakeWristMotor.getVelocity();
    intakeWristAppliedVolts = intakeWristMotor.getMotorVoltage();
    intakeWristStatorCurrent = intakeWristMotor.getStatorCurrent();
    intakeWristSupplyCurrent = intakeWristMotor.getSupplyCurrent();
    intakeWristTemperature = intakeWristMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeWristPosition,
        intakeWristVelocity,
        intakeWristAppliedVolts,
        intakeWristStatorCurrent,
        intakeWristSupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(5, intakeWristTemperature);

    PhoenixUtil.tryUntilOk(
        10, () -> intakeWristMotor.getConfigurator().apply(intakeWristConfig, 1));

    intakeWristMotor.optimizeBusUtilization();
  }

  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        intakeWristPosition,
        intakeWristVelocity,
        intakeWristAppliedVolts,
        intakeWristStatorCurrent,
        intakeWristSupplyCurrent,
        intakeWristTemperature);

    super.currentPosition = intakeWristPosition.getValueAsDouble();
    super.velocity = intakeWristVelocity.getValueAsDouble();
    super.voltage = intakeWristAppliedVolts.getValueAsDouble();
    super.statorCurrent = intakeWristStatorCurrent.getValueAsDouble();
    super.supplyCurrent = intakeWristSupplyCurrent.getValueAsDouble();
    super.temperature = intakeWristTemperature.getValueAsDouble();

    super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;
    super.isWristJammed =
        wristStallDebouncer.calculate(
            (super.statorCurrent > IntakeWristConstants.agitateStatorCurrentTrip)
                && (Math.abs(super.velocity) < IntakeWristConstants.agitateVelocityTrip));

    DogLog.log("Intake/Wrist/Position", super.currentPosition);
    DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
    DogLog.log("Intake/Wrist/Velocity", super.velocity);
    DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
    DogLog.log("Intake/Wrist/Voltage", super.voltage);
    DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
    DogLog.log("Intake/Wrist/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Intake/Wrist/Temperature", super.temperature);
  }

  @Override
  public void stopIntakeWrist() {
    intakeWristMotor.setVoltage(0);
  }

  @Override
  public void setIntakeWristPosition(double position) {
    super.targetPosition = position;
    intakeWristMotor.setControl(m_request.withPosition(position).withEnableFOC(true));
  }

  @Override
  public void setIntakeWristDutyCycle(double dutyCycle) {
    intakeWristMotor.setControl(dutyCycleRequest.withOutput(dutyCycle).withEnableFOC(true));
  }

  @Override
  public double getIntakeWristPosition() {
    return intakeWristPosition.getValueAsDouble();
  }

  @Override
  public void homeIntakeWrist() {
    intakeWristMotor.setPosition(0.0);
  }

  @Override
  public boolean isWristReadyToHome() {
    return (Math.abs(super.velocity) < IntakeWristConstants.homingVelocityThreshold)
        && (super.statorCurrent > IntakeWristConstants.homingCurrentThreshold);
  }
}
