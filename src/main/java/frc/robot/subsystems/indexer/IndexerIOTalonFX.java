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
  private final TalonFX indexerMotor;
  private final CANBus indexerCANBus;
  private TalonFXConfiguration indexerConfig;

  private final Timer pulseTimer = new Timer();
  private boolean pulseOn = false;

  private final StatusSignal<AngularVelocity> indexerVelocityRad;
  private final StatusSignal<Temperature> indexerTemperature;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerStatorCurrent;
  private final StatusSignal<Current> indexerSupplyCurrent;

  public IndexerIOTalonFX() {
    indexerCANBus = new CANBus(IndexerConstants.indexerCANBus);
    indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, indexerCANBus);

    indexerConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IndexerConstants.statorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.supplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));

    PhoenixUtil.tryUntilOk(10, () -> indexerMotor.getConfigurator().apply(indexerConfig, 1));

    indexerVelocityRad = indexerMotor.getVelocity();
    indexerTemperature = indexerMotor.getDeviceTemp();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();

    // BaseStatusSignal.setUpdateFrequencyForAll(50, indexerVelocityRad,
    //     indexerTemperature, indexerAppliedVolts, indexerStatorCurrent,
    //     indexerSupplyCurrent);

    //indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    // BaseStatusSignal.refreshAll(indexerVelocityRad, indexerTemperature,
    //     indexerAppliedVolts, indexerStatorCurrent, indexerSupplyCurrent);

    super.velocity = indexerVelocityRad.getValueAsDouble();
    super.supplyCurrent = indexerSupplyCurrent.getValueAsDouble();
    super.statorCurrent = indexerStatorCurrent.getValueAsDouble();
    super.appliedVolts = indexerAppliedVolts.getValueAsDouble();
    super.tempCelsius = indexerTemperature.getValueAsDouble();

    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Indexer/StatorCurrent", super.statorCurrent);
    DogLog.log("Indexer/AppliedVolts", super.appliedVolts);
    DogLog.log("Indexer/Temperature", super.tempCelsius);
  }

  @Override
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    indexerMotor.setVoltage(volts);
  }

  @Override
  public void indexerPulseFuel(double volts) {
    if (!pulseTimer.isRunning()) {
      pulseTimer.start();
    }

    if (pulseTimer.get() > IndexerConstants.pulseTimeInterval) {
      if (!pulseOn) {
        indexerMotor.setVoltage(volts);
      } else {
        indexerMotor.setVoltage(0);
      }
      pulseOn = !pulseOn;
      pulseTimer.reset();
    }
  }
}