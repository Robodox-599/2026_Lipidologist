package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
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
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final MotionMagicVoltage motionMagicRequest;
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbStatorCurrent;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  public ClimbIOTalonFX() {
    leaderMotor = new TalonFX(ClimbConstants.climbLeaderMotorID, ClimbConstants.climbFollowerMotorCANbus);
    followerMotor = new TalonFX(ClimbConstants.climbFollowerMotorID, ClimbConstants.climbFollowerMotorCANbus);

    //followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.maxVelocityRotsPerSec;
    config.MotionMagic.MotionMagicAcceleration = ClimbConstants.maxAccelerationRotationsPerSecSQ;
 
    config.Slot0.kP = ClimbConstants.kP;
    config.Slot0.kI = ClimbConstants.kI;
    config.Slot0.kD = ClimbConstants.kD;
    config.Slot0.kV = ClimbConstants.kV;
    config.Slot0.kS = ClimbConstants.kS;
    config.Slot0.kG = ClimbConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimitAmps;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(10, () -> leaderMotor.getConfigurator().apply(config, 1));
    PhoenixUtil.tryUntilOk(10, () -> followerMotor.getConfigurator().apply(config, 1));

    climbPosition = leaderMotor.getPosition();
    climbVelocity = leaderMotor.getVelocity();
    climbAppliedVolts = leaderMotor.getMotorVoltage();
    climbStatorCurrent = leaderMotor.getStatorCurrent();
    climbSupplyCurrent = leaderMotor.getSupplyCurrent();
    climbTempCelsius = leaderMotor.getDeviceTemp();
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, climbVelocity, climbTempCelsius, 
          climbPosition, climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts);

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();

    zeroClimbEncoder();
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
  public void setClimbHeight(double height) {
    double position = 
      MathUtil.clamp
        (ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit, ClimbConstants.climbUpperLimit);

    motionMagicRequest.Position = position;
    leaderMotor.setControl(motionMagicRequest);
  }

  @Override
  public void stopClimb() {
    leaderMotor.stopMotor();
  }

  @Override
  public void setClimbVoltage(double voltage) {
    leaderMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void zeroClimbEncoder() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }
}