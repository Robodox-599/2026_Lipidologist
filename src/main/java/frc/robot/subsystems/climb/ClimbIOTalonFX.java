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
  private TalonFX climbMotor;
  private CANBus climbCanbus;
  private TalonFXConfiguration climbConfig;
  private StatusSignal<Angle> climbPosition;
  private StatusSignal<AngularVelocity> climbVelocity;
  private StatusSignal<Current> climbStatorCurrent;
  private StatusSignal<Current> climbSupplyCurrent;
  private StatusSignal<Voltage> climbVoltage;
  private StatusSignal<Temperature> climbTemperature;

  public void ClimbIOTalonFX(){
    climbCanbus = new CANBus(ClimbConstants.climbCanbus);
    climbMotor = new TalonFX(ClimbConstants.climbMotorID, climbCanbus);
    climbConfig = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(ClimbConstants.statorCurrent)
          .withSupplyCurrentLimit(ClimbConstants.supplyCurrent)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withSlot0(
        new Slot0Configs()
          .withKP(ClimbConstants.kP)
          .withKI(ClimbConstants.kI)
          .withKD(ClimbConstants.kD)
          .withKV(ClimbConstants.kV)
          .withKS(ClimbConstants.kS)
      );

      climbPosition = climbMotor.getPosition();
      climbVelocity = climbMotor.getVelocity();
      climbStatorCurrent = climbMotor.getStatorCurrent();
      climbSupplyCurrent = climbMotor.getSupplyCurrent();
      climbVoltage = climbMotor.getMotorVoltage();
      climbTemperature = climbMotor.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(50, climbPosition, climbVelocity,
         climbStatorCurrent, climbSupplyCurrent, climbVoltage, climbTemperature);

      climbMotor.optimizeBusUtilization();
  }

  public void updateInputs(){
    BaseStatusSignal.refreshAll(climbPosition, climbVelocity, climbStatorCurrent, climbSupplyCurrent, climbVoltage, climbTemperature);

    super.position = climbPosition.getValueAsDouble();
    super.velocity = climbVelocity.getValue
    
  }


}