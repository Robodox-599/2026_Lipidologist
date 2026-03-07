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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;


public class ClimbIOTalonFX extends ClimbIO {
  private TalonFX climbMotor;
  private CANBus climbCanbus;
  private TalonFXConfiguration climbConfig;


  private MotionMagicVoltage m_request;
  private VoltageOut v_request;


  private StatusSignal<Angle> climbPosition;
  private StatusSignal<Current> climbStatorCurrent;
  private StatusSignal<Current> climbSupplyCurrent;
  private StatusSignal<Voltage> climbVoltage;
  private StatusSignal<Temperature> climbTemperature;


  public ClimbIOTalonFX(){
    climbCanbus = new CANBus(ClimbConstants.climbCanbus);
    climbMotor = new TalonFX(ClimbConstants.climbMotorID, climbCanbus);
    climbConfig = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withStatorCurrentLimit(ClimbConstants.statorCurrent)
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(ClimbConstants.supplyCurrent)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withSlot0(
        new Slot0Configs()
          .withKP(ClimbConstants.kP)
          .withKI(ClimbConstants.kI)
          .withKD(ClimbConstants.kD)
          .withKV(ClimbConstants.kV)
          .withKS(ClimbConstants.kS)
          .withKG(ClimbConstants.kG)
          .withGravityType(GravityTypeValue.Elevator_Static)
      )
      .withMotionMagic(
        new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(ClimbConstants.maxVelocity)
          .withMotionMagicAcceleration(ClimbConstants.maxAcceleration)
      )
      
      ;
    m_request = new MotionMagicVoltage(0);
    v_request = new VoltageOut(0);


    PhoenixUtil.tryUntilOk(10, () -> climbMotor.getConfigurator().apply(climbConfig));


      climbPosition = climbMotor.getPosition();
      climbStatorCurrent = climbMotor.getStatorCurrent();
      climbSupplyCurrent = climbMotor.getSupplyCurrent();
      climbVoltage = climbMotor.getMotorVoltage();
      climbTemperature = climbMotor.getDeviceTemp();


      BaseStatusSignal.setUpdateFrequencyForAll(50, climbPosition,
         climbStatorCurrent, climbSupplyCurrent, climbVoltage, climbTemperature);


      climbMotor.optimizeBusUtilization();
  }


  @Override
  public void updateInputs(){
    BaseStatusSignal.refreshAll(climbPosition, climbStatorCurrent, climbSupplyCurrent, climbVoltage, climbTemperature);


    super.position = climbPosition.getValueAsDouble();
    super.statorCurrent = climbStatorCurrent.getValueAsDouble();
    super.supplyCurrent = climbSupplyCurrent.getValueAsDouble();
    super.voltage = climbVoltage.getValueAsDouble();
    super.temperature = climbTemperature.getValueAsDouble();


    super.atSetpoint = Math.abs(super.targetPosition - super.position) < 0.02;


    DogLog.log("Climb/Position", super.position);
    DogLog.log("Climb/Velocity", super.velocity);
    DogLog.log("Climb/StatorCurrent", super.statorCurrent);
    DogLog.log("Climb/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Climb/Voltage", super.voltage);
    DogLog.log("Climb/Temperature", super.temperature);
    DogLog.log("Climb/AtSetpoint", super.atSetpoint);
  }

  @Override
  public void setPosition(double position){
    super.targetPosition = position;
    climbMotor.setControl(m_request.withPosition(position));
  }

  @Override
  public void stop(){
    climbMotor.stopMotor();
  }

  @Override
  public void zeroClimb(){
    if (super.statorCurrent > ClimbConstants.tripStatorCurrent){
      setPosition(0); 
    } else{
      climbMotor.setControl(v_request.withOutput(-1)); //voltage change may be needed
    }
  }
}
