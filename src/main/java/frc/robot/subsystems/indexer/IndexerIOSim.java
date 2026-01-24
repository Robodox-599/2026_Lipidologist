// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim extends IndexerIO {
  private final DCMotorSim indexerSimMotor;
  private static final DCMotor indexerGearbox = DCMotor.getKrakenX60Foc(1);

  private final Timer pulseTimer = new Timer();
  private boolean pulseOn = false;

  /** Creates a new IndexerIOSim. */
  public IndexerIOSim() {
    indexerSimMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (indexerGearbox, IndexerConstants.indexerMOI, IndexerConstants.indexerGearRatio), indexerGearbox);
  }

  @Override
  public void updateInputs(){
    indexerSimMotor.update(0.02);

    super.position = indexerSimMotor.getAngularPositionRad();
    super.velocity = indexerSimMotor.getAngularVelocityRPM() / 60.0;
    super.statorCurrent = indexerSimMotor.getCurrentDrawAmps();
    super.supplyCurrent = indexerSimMotor.getCurrentDrawAmps();
    super.appliedVolts = indexerSimMotor.getInputVoltage();
    super.tempCelsius = 25.0;

    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/Position", super.position);
    DogLog.log("Indexer/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Indexer/StatorCurrent", super.statorCurrent);
    DogLog.log("Indexer/AppliedVolts", super.appliedVolts);
    DogLog.log("Indexer/Temperature", super.tempCelsius);
  }

  @Override
  public void stopIndexer(){
    indexerSimMotor.setInputVoltage(0);
  }

  @Override
  public void setIndexerVoltage(double volts){
    indexerSimMotor.setInputVoltage(volts);
  }

  @Override
    public void indexerPulseFuel(double volts) {
      if(!pulseTimer.isRunning()){
        pulseTimer.start();
      }

      if (pulseTimer.get() > IndexerConstants.pulseTimeInterval) {
        if(!pulseOn){
          indexerSimMotor.setInputVoltage(volts);
        } else{
          indexerSimMotor.setInputVoltage(0);
        }
        pulseOn = !pulseOn;
        pulseTimer.reset();
      }
    }
  
}
