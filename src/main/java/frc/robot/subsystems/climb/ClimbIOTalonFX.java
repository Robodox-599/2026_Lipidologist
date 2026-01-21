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

  private final StatusSignal<Angle> leftClimbMotorPosition;
  private final StatusSignal<AngularVelocity> leftClimbMotorVelocity;
  private final StatusSignal<Voltage> leftClimbMotorAppliedVolts;
  private final StatusSignal<Current> leftClimbMotorStatorCurrent;
  private final StatusSignal<Current> leftClimbMotorSupplyCurrent;
  private final StatusSignal<Temperature> leftClimbMotorTempCelsius;

  private final StatusSignal<Angle> rightClimbMotorPosition;
  private final StatusSignal<AngularVelocity> rightClimbMotorVelocity;
  private final StatusSignal<Voltage> rightClimbMotorAppliedVolts;
  private final StatusSignal<Current> rightClimbMotorStatorCurrent;
  private final StatusSignal<Current> rightClimbMotorSupplyCurrent;
  private final StatusSignal<Temperature> rightClimbMotorTempCelsius;

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

    leftClimbMotorPosition = leftMotor.getPosition();
    leftClimbMotorVelocity = leftMotor.getVelocity();
    leftClimbMotorAppliedVolts = leftMotor.getMotorVoltage();
    leftClimbMotorStatorCurrent = leftMotor.getStatorCurrent();
    leftClimbMotorSupplyCurrent = leftMotor.getSupplyCurrent();
    leftClimbMotorTempCelsius = leftMotor.getDeviceTemp();

    rightClimbMotorPosition = rightMotor.getPosition();
    rightClimbMotorVelocity = rightMotor.getVelocity();
    rightClimbMotorAppliedVolts = rightMotor.getMotorVoltage();
    rightClimbMotorStatorCurrent = rightMotor.getStatorCurrent();
    rightClimbMotorSupplyCurrent = rightMotor.getSupplyCurrent();
    rightClimbMotorTempCelsius = rightMotor.getDeviceTemp();
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leftClimbMotorVelocity, leftClimbMotorTempCelsius, leftClimbMotorPosition, 
        leftClimbMotorStatorCurrent, leftClimbMotorSupplyCurrent, leftClimbMotorAppliedVolts, leftClimbMotorTempCelsius,
          rightClimbMotorVelocity, rightClimbMotorTempCelsius, rightClimbMotorPosition, rightClimbMotorStatorCurrent, 
            rightClimbMotorSupplyCurrent, rightClimbMotorAppliedVolts, rightClimbMotorTempCelsius);

    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();

    zeroClimbPosition();
  }

  @Override
  public void updateClimbInputs() {
    BaseStatusSignal.refreshAll
      (leftClimbMotorVelocity, leftClimbMotorTempCelsius, leftClimbMotorPosition, 
        leftClimbMotorStatorCurrent, leftClimbMotorSupplyCurrent, leftClimbMotorAppliedVolts, leftClimbMotorTempCelsius,
          rightClimbMotorVelocity, rightClimbMotorTempCelsius, rightClimbMotorPosition, rightClimbMotorStatorCurrent, 
            rightClimbMotorSupplyCurrent, rightClimbMotorAppliedVolts, rightClimbMotorTempCelsius);

    // left motor
    super.leftMotorPositionInches = leftClimbMotorPosition.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.leftMotorTargetPositionInches = motionMagicRequest.Position * ClimbConstants.inchesPerRev;
    super.leftMotorVelocityInchesPerSec = leftClimbMotorVelocity.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.leftMotorAppliedVolts = leftClimbMotorAppliedVolts.getValueAsDouble();
    super.leftMotorStatorCurrent = leftClimbMotorStatorCurrent.getValueAsDouble();
    super.leftMotorSupplyCurrent = leftClimbMotorSupplyCurrent.getValueAsDouble();
    super.leftMotorTempCelsius = leftClimbMotorTempCelsius.getValueAsDouble();

    // right motor
    super.rightMotorPositionInches = rightClimbMotorPosition.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.rightMotorTargetPositionInches = motionMagicRequest.Position * ClimbConstants.inchesPerRev;
    super.rightMotorVelocityInchesPerSec = rightClimbMotorVelocity.getValueAsDouble() * ClimbConstants.inchesPerRev;
    super.rightMotorAppliedVolts = rightClimbMotorAppliedVolts.getValueAsDouble();
    super.rightMotorStatorCurrent = rightClimbMotorStatorCurrent.getValueAsDouble();
    super.rightMotorSupplyCurrent = rightClimbMotorSupplyCurrent.getValueAsDouble();
    super.rightMotorTempCelsius = rightClimbMotorTempCelsius.getValueAsDouble();

    // left motor
    DogLog.log("Climb/LeftPositionInches", super.leftMotorPositionInches);
    DogLog.log("Climb/LeftTargetPositionInches", super.leftMotorTargetPositionInches);
    DogLog.log("Climb/LeftVelocityInchesPerSec", super.leftMotorVelocityInchesPerSec);
    DogLog.log("Climb/LeftStatorCurrent", super.leftMotorStatorCurrent);
    DogLog.log("Climb/LeftSupplyCurrent", super.leftMotorSupplyCurrent);
    DogLog.log("Climb/LeftAppliedVoltage", super.leftMotorAppliedVolts);
    DogLog.log("Climb/LeftTempCelcius", super.leftMotorTempCelsius);

    // right motor
    DogLog.log("Climb/RightPositionInches", super.rightMotorPositionInches);
    DogLog.log("Climb/RightTargetPositionInches", super.rightMotorTargetPositionInches);
    DogLog.log("Climb/RightVelocityInchesPerSec", super.rightMotorVelocityInchesPerSec);
    DogLog.log("Climb/RightStatorCurrent", super.rightMotorStatorCurrent);
    DogLog.log("Climb/RightSupplyCurrent", super.rightMotorSupplyCurrent);
    DogLog.log("Climb/RightAppliedVoltage", super.rightMotorAppliedVolts);
    DogLog.log("Climb/RightTempCelcius", super.rightMotorTempCelsius);
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