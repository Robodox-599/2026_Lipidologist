package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Motor.TalonFXWrapper;

public class IndexerIOTalonFX extends IndexerIO {
  private final TalonFXWrapper indexerMotorWrapper;
  private final TalonFX indexerMotor;
  private final CANBus indexerCANBus;

  private final StatusSignal<AngularVelocity> indexerVelocityRad;
  private final StatusSignal<Temperature> indexerTemperature;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerStatorCurrent;
  private final StatusSignal<Current> indexerSupplyCurrent;

  private final VoltageOut voltageOut;

  public IndexerIOTalonFX() {
    indexerCANBus = new CANBus(IndexerConstants.indexerMotor.canBus());

    indexerMotorWrapper = new TalonFXWrapper(IndexerConstants.indexerMotor);
    indexerMotor = indexerMotorWrapper.getTalonFX();

    voltageOut = new VoltageOut(0);

    indexerVelocityRad = indexerMotor.getVelocity();
    indexerTemperature = indexerMotor.getDeviceTemp();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        indexerVelocityRad,
        indexerTemperature,
        indexerAppliedVolts,
        indexerStatorCurrent,
        indexerSupplyCurrent);

    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        indexerVelocityRad,
        indexerTemperature,
        indexerAppliedVolts,
        indexerStatorCurrent,
        indexerSupplyCurrent);

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
    indexerMotorWrapper.stop();
  }

  @Override
  public void setIndexerVoltage(double voltage) {
    indexerMotor.setControl(voltageOut.withOutput(voltage).withEnableFOC(true));
  }
}
