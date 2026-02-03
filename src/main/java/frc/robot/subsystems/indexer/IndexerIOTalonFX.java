package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;         
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class IndexerIOTalonFX extends IndexerIO {
    private final TalonFX indexerMotor;
    private final CANBus indexerBus;
    private TalonFXConfiguration indexerMotorConfig;

    private final Timer pulseTimer = new Timer();
    private boolean pulseOn = false;

    private final StatusSignal<AngularVelocity> indexerVelocityRad;
    private final StatusSignal<Temperature> indexerTemperature;
    private final StatusSignal<Angle> indexerPosition;
    private final StatusSignal<Voltage> indexerAppliedVolts;
    private final StatusSignal<Current> indexerStatorCurrent;
    private final StatusSignal<Current> indexerSupplyCurrent;

    public IndexerIOTalonFX() {
        indexerBus = new CANBus(IndexerConstants.indexerCANBus);
        indexerMotor = new TalonFX(IndexerConstants.indexerMotorID, indexerBus);
        indexerMotorConfig = new TalonFXConfiguration();
        
        indexerMotorConfig.Slot0.kP = IndexerConstants.kP;
        indexerMotorConfig.Slot0.kI = IndexerConstants.kI;
        indexerMotorConfig.Slot0.kD = IndexerConstants.kD;
        indexerMotorConfig.Slot0.kS = IndexerConstants.kS;
        indexerMotorConfig.Slot0.kV = IndexerConstants.kV;

        indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.supplyCurrentLimit;

        indexerMotor.getConfigurator().apply(indexerMotorConfig);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerVelocityRad = indexerMotor.getVelocity();
        indexerTemperature = indexerMotor.getDeviceTemp();
        indexerAppliedVolts = indexerMotor.getMotorVoltage();
        indexerPosition = indexerMotor.getPosition();
        indexerStatorCurrent = indexerMotor.getStatorCurrent();
        indexerSupplyCurrent = indexerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,indexerVelocityRad, 
            indexerTemperature, indexerAppliedVolts, indexerPosition, indexerStatorCurrent, 
                indexerSupplyCurrent);

        indexerMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateIndexerInputs() {
        BaseStatusSignal.refreshAll(indexerVelocityRad, indexerTemperature, 
            indexerAppliedVolts, indexerPosition, indexerStatorCurrent, indexerSupplyCurrent);

        super.position = indexerPosition.getValueAsDouble();
        super.velocity = indexerVelocityRad.getValueAsDouble();
        super.supplyCurrent = indexerSupplyCurrent.getValueAsDouble();
        super.statorCurrent = indexerStatorCurrent.getValueAsDouble();
        super.appliedVolts = indexerAppliedVolts.getValueAsDouble();
        super.tempCelsius = indexerTemperature.getValueAsDouble();

        DogLog.log("Indexer/Velocity", super.velocity);
        DogLog.log("Indexer/Position", super.position);
        DogLog.log("Indexer/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Indexer/StatorCurrent", super.statorCurrent);
        DogLog.log("Indexer/AppliedVolts", super.appliedVolts);
        DogLog.log("Indexer/Temperature", super.tempCelsius);
    }

    @Override
    public void stopIndexer() {
      indexerMotor.stopMotor();
    }

    @Override
    public void setIndexerVoltage(double volts) {
      indexerMotor.setVoltage(volts);
    }

    @Override
    public void indexerPulseFuel(double volts) {
      if(!pulseTimer.isRunning()){
        pulseTimer.start();
      }

      if (pulseTimer.get() > IndexerConstants.pulseTimeInterval) {
        if(!pulseOn){
          indexerMotor.setVoltage(volts);
        } else{
          indexerMotor.setVoltage(0);
        }
        pulseOn = !pulseOn;
        pulseTimer.reset();
      }
    }
}