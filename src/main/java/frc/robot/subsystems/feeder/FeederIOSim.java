// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIO {
  private final DCMotorSim feederSimMotor;

  /** Creates a new IndexerIOSim. */
  public FeederIOSim() {
    feederSimMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60Foc(1), FeederConstants.feederMOI, 
        FeederConstants.feederGearRatio), DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateFeederInputs(){
    feederSimMotor.update(0.02);

    super.position = feederSimMotor.getAngularPositionRad();
    super.velocity = feederSimMotor.getAngularVelocityRPM() / 60.0;
    super.statorCurrent = feederSimMotor.getCurrentDrawAmps();
    super.supplyCurrent = feederSimMotor.getCurrentDrawAmps();
    super.appliedVolts = feederSimMotor.getInputVoltage();
    super.tempCelsius = 25.0;

    DogLog.log("Feeder/Velocity", super.velocity);
    DogLog.log("Feeder/Position", super.position);
    DogLog.log("Feeder/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Feeder/StatorCurrent", super.statorCurrent);
    DogLog.log("Feeder/AppliedVolts", super.appliedVolts);
    DogLog.log("Feeder/Temperature", super.tempCelsius);
  }

  @Override
  public void stopFeeder(){
    feederSimMotor.setInputVoltage(0);
  }

  @Override
  public void setFeederVoltage(double volts){
    feederSimMotor.setInputVoltage(volts);
  }  
}
