package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX extends ClimbIO {
  private final TalonFX climbMotor;
  private final CANBus climbBus;
  private TalonFXConfiguration climbConfig;

  private final MotionMagicVoltage motionMagicRequest;

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbStatorCurrent;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  public ClimbIOTalonFX() {
    climbBus = new CANBus(ClimbConstants.climbMotorCANbus);
    climbMotor = new TalonFX(ClimbConstants.climbMotorID, climbBus);

    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    climbConfig = new TalonFXConfiguration()
        .withMotionMagic(
          new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ClimbConstants.maxVelocityRotsPerSec)
            .withMotionMagicAcceleration(ClimbConstants.maxAccelerationRotationsPerSecSQ))
        .withSlot0(
          new Slot0Configs()
            .withKP(ClimbConstants.kP)
            .withKI(ClimbConstants.kI)
            .withKD(ClimbConstants.kD)
            .withKV(ClimbConstants.kV)
            .withKS(ClimbConstants.kS)
            .withKG(ClimbConstants.kG)
            .withGravityType(GravityTypeValue.Elevator_Static))
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ClimbConstants.statorCurrentLimitAmps)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ClimbConstants.supplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));

    PhoenixUtil.tryUntilOk(10, () -> climbMotor.getConfigurator().apply(climbConfig, 1));

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbStatorCurrent = climbMotor.getStatorCurrent();
    climbSupplyCurrent = climbMotor.getSupplyCurrent();
    climbTempCelsius = climbMotor.getDeviceTemp();

    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0, climbVelocity, climbPosition,
    //     climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts,
    //     climbTempCelsius);

    // climbMotor.optimizeBusUtilization();

    zeroClimbPosition();
  }

  @Override
  public void updateInputs() {
    // BaseStatusSignal.refreshAll(climbVelocity, climbTempCelsius, climbPosition,
    //     climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts);

    super.positionInches = climbPosition.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.targetPositionInches = motionMagicRequest.Position * ClimbConstants.inchesPerRev;
    super.velocityInchesPerSec = climbVelocity.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.appliedVolts = climbAppliedVolts.getValueAsDouble();
    super.statorCurrent = climbStatorCurrent.getValueAsDouble();
    super.supplyCurrent = climbSupplyCurrent.getValueAsDouble();
    super.tempCelsius = climbTempCelsius.getValueAsDouble();

    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Climb/StatorCurrent", super.statorCurrent);
    DogLog.log("Climb/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Climb/AppliedVoltage", super.appliedVolts);
    DogLog.log("Climb/TempCelcius", super.tempCelsius);
  }

  /*
   * This method is used to make the motor go to whatever position we set it to in
   * the command layer, while
   * also clamping the range of positions it can go to
   * 
   * @param height A double representing a quantity of inches.
   */
  @Override
  public void setClimbHeight(double height) {
    double position = MathUtil.clamp(ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit,
        ClimbConstants.climbUpperLimit);

    motionMagicRequest.Position = position;
    climbMotor.setControl(motionMagicRequest);
  }

  @Override
  public void stopClimb() {
    climbMotor.setVoltage(0);
  }

  @Override
  public void setClimbVoltage(double voltage) {
    climbMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void zeroClimbPosition() {
    climbMotor.setPosition(0);
  }
}