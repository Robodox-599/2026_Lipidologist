package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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

    climbConfig = new TalonFXConfiguration();

    climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climbConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.maxVelocityRotsPerSec;
    climbConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.maxAccelerationRotationsPerSecSQ;
 
    climbConfig.Slot0.kP = ClimbConstants.kP;
    climbConfig.Slot0.kI = ClimbConstants.kI;
    climbConfig.Slot0.kD = ClimbConstants.kD;
    climbConfig.Slot0.kV = ClimbConstants.kV;
    climbConfig.Slot0.kS = ClimbConstants.kS;
    climbConfig.Slot0.kG = ClimbConstants.kG;
    climbConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.statorCurrentLimitAmps;
    climbConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimitAmps;
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(10, () -> climbMotor.getConfigurator().apply(climbConfig, 1));

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbStatorCurrent = climbMotor.getStatorCurrent();
    climbSupplyCurrent = climbMotor.getSupplyCurrent();
    climbTempCelsius = climbMotor.getDeviceTemp();        
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, climbVelocity, climbPosition, 
        climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts, 
          climbTempCelsius);

    climbMotor.optimizeBusUtilization();

    zeroClimbPosition();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll
      (climbVelocity, climbTempCelsius, climbPosition, 
        climbStatorCurrent, climbSupplyCurrent, climbAppliedVolts);

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
   * This method is used to make the motor go to whatever position we set it to in the command layer, while
   * also clamping the range of positions it can go to
   * 
   * @param height A double representing a quantity of inches.
  */
  @Override
  public void setClimbHeight(double height) {
    double position = 
      MathUtil.clamp
        (ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit, ClimbConstants.climbUpperLimit);

    motionMagicRequest.Position = position;
    climbMotor.setControl(motionMagicRequest);
  }

  @Override
  public void stopClimb() {
    climbMotor.stopMotor();
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