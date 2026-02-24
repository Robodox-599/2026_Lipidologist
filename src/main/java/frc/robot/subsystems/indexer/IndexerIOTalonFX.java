package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX extends IndexerIO {
  private CANBus canbus;
  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfig;

  private StatusSignal<AngularVelocity> indexerVelocity;
  private StatusSignal<Current> indexerStatorCurrent;
  private StatusSignal<Current> indexerSupplyCurrent;
  private StatusSignal<Voltage> indexerVoltage;
  private StatusSignal<Temperature> indexerTemperature;

  public IndexerIOTalonFX(){
    canbus = new CANBus(IndexerConstants.indexerCanbus);
    indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, canbus);
    indexerConfig = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(IndexerConstants.statorCurrent)
          .withSupplyCurrentLimit(IndexerConstants.supplyCurrent)
      ).withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
      );
    
    PhoenixUtil.tryUntilOk(10, () -> indexerMotor.getConfigurator().apply(indexerConfig));

    indexerVelocity = indexerMotor.getVelocity();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent= indexerMotor.getSupplyCurrent();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerTemperature = indexerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50, indexerVelocity, indexerStatorCurrent, indexerSupplyCurrent, indexerVoltage, indexerTemperature);
    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(){
    BaseStatusSignal.refreshAll(indexerVelocity, indexerStatorCurrent, indexerSupplyCurrent, indexerVoltage, indexerTemperature);

    super.velocity = indexerVelocity.getValueAsDouble();
    super.statorCurrent = indexerStatorCurrent.getValueAsDouble();
    super.supplyCurrent = indexerSupplyCurrent.getValueAsDouble();
    super.voltage = indexerVoltage.getValueAsDouble();
    super.temperature = indexerTemperature.getValueAsDouble();

    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/StatorCurrent", super.statorCurrent);
    DogLog.log("Indexer/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Indexer/Voltage", super.voltage);
    DogLog.log("Indexer/Temperature", super.temperature);
  }

  @Override
  public void stop(){
    indexerMotor.setVoltage(0);
  }

  @Override
  public void setVoltage(double voltage){
    indexerMotor.setVoltage(voltage);
  }
}