package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final MotionMagicVoltage motionMagicRequest;
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbStatorCurrent;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  public ClimbIOTalonFX() {
    leftMotor = new TalonFX(ClimbConstants.climbLeaderMotorID, ClimbConstants.climbFollowerMotorCANbus);
    rightMotor = new TalonFX(ClimbConstants.climbFollowerMotorID, ClimbConstants.climbFollowerMotorCANbus);

    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

    // left motor config

    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.maxVelocityRotsPerSec;
    leftMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.maxAccelerationRotationsPerSecSQ;
 
    leftMotorConfig.Slot0.kP = ClimbConstants.kP;
    leftMotorConfig.Slot0.kI = ClimbConstants.kI;
    leftMotorConfig.Slot0.kD = ClimbConstants.kD;
    leftMotorConfig.Slot0.kV = ClimbConstants.kV;
    leftMotorConfig.Slot0.kS = ClimbConstants.kS;
    leftMotorConfig.Slot0.kG = ClimbConstants.kG;
    leftMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leftMotorConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorCurrentLimitAmps;
    leftMotorConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimitAmps;
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // right motor config

    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.maxVelocityRotsPerSec;
    rightMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.maxAccelerationRotationsPerSecSQ;
 
    rightMotorConfig.Slot0.kP = ClimbConstants.kP;
    rightMotorConfig.Slot0.kI = ClimbConstants.kI;
    rightMotorConfig.Slot0.kD = ClimbConstants.kD;
    rightMotorConfig.Slot0.kV = ClimbConstants.kV;
    rightMotorConfig.Slot0.kS = ClimbConstants.kS;
    rightMotorConfig.Slot0.kG = ClimbConstants.kG;
    rightMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    rightMotorConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorCurrentLimitAmps;
    rightMotorConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimitAmps;
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(10, () -> leftMotor.getConfigurator().apply(leftMotorConfig, 1));
    PhoenixUtil.tryUntilOk(10, () -> rightMotor.getConfigurator().apply(rightMotorConfig, 1));

    climbPosition = leftMotor.getPosition();
    climbVelocity = leftMotor.getVelocity();
    climbAppliedVolts = leftMotor.getMotorVoltage();
    climbStatorCurrent = leftMotor.getStatorCurrent();
    climbSupplyCurrent = leftMotor.getSupplyCurrent();
    climbTempCelsius = leftMotor.getDeviceTemp();
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, climbVelocity, climbTempCelsius, 
          climbPosition, climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts);

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();

    zeroClimbPosition();
  }

  @Override
  public void updateClimbInputs() {
    BaseStatusSignal.refreshAll
      (climbVelocity, climbTempCelsius, climbPosition, climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts);

    super.positionInches = climbPosition.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.targetPositionInches = motionMagicRequest.Position * ClimbConstants.inchesPerRev;
    super.velocityInchesPerSec = climbVelocity.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.appliedVolts = climbAppliedVolts.getValueAsDouble();
    super.statorCurrent = climbStatorCurrent.getValueAsDouble();
    super.supplyCurrent = climbSupplyCurrent.getValueAsDouble();

    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Climb/StatorCurrent", super.statorCurrent);
    DogLog.log("Climb/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Climb/AppliedVoltage", super.appliedVolts);
    DogLog.log("Climb/TempCelcius", super.tempCelsius);
  }

  @Override
  public void setLeftClimbHeight(double height) {
    double position = 
      MathUtil.clamp
        (ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit, ClimbConstants.climbUpperLimit);

    motionMagicRequest.Position = position;
    leftMotor.setControl(motionMagicRequest);
  }

  @Override
  public void setRightClimbHeight(double height) {
    double position = 
      MathUtil.clamp
        (ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit, ClimbConstants.climbUpperLimit);

    motionMagicRequest.Position = position;
    rightMotor.setControl(motionMagicRequest);
  }

  @Override
  public void stopClimb() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void setLeftClimbVoltage(double voltage) {
    leftMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setRightClimbVoltage(double voltage) {
    rightMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void zeroClimbPosition() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }
}