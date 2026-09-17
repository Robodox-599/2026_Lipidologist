// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIO {
  private final DCMotorSim feederSimMotor;

  public FeederIOSim() {
    feederSimMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                FeederConstants.feederMOI,
                FeederConstants.feederMotor.gearRatio()),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs() {
    feederSimMotor.update(0.02);

    super.RPS = feederSimMotor.getAngularVelocityRPM() / 60.0;
    super.tempCelsius = 25.0;

    DogLog.log("Feeder/RPS", super.RPS);
    DogLog.log("Feeder/TargetRPS", super.targetRPS);
    DogLog.log("Feeder/Temperature", super.tempCelsius);
  }

  @Override
  public void stopFeeder() {
    super.targetRPS = 0.0;
    feederSimMotor.setInputVoltage(0.0);
  }

  @Override
  public void setFeederVelocity(double RPS) {
    super.targetRPS = RPS;
    feederSimMotor.setAngularVelocity(RPS * 2.0 * Math.PI);
  }

  @Override
  public void setFeederVoltage(double voltage) {
    feederSimMotor.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }
}
