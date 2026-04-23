// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeRollersIOSim extends IntakeRollersIO {
  private final DCMotorSim intakeRollersLeaderSim;
  private final DCMotorSim intakeRollersFollowerSim;

  public IntakeRollersIOSim() {
    intakeRollersLeaderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                IntakeRollersConstants.rotationalInertia,
                IntakeRollersConstants.intakeRollersGearRatio),
            DCMotor.getKrakenX60Foc(1));
    intakeRollersFollowerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                IntakeRollersConstants.rotationalInertia,
                IntakeRollersConstants.intakeRollersGearRatio),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void updateInputs() {
    // intake rollers 1
    intakeRollersLeaderSim.update(0.02);
    super.intakeRollersLeaderStatorCurrent = intakeRollersLeaderSim.getCurrentDrawAmps();
    super.intakeRollersLeaderVelocity = intakeRollersLeaderSim.getAngularVelocityRPM();
    super.intakeRollersLeaderVoltage = intakeRollersLeaderSim.getInputVoltage();
    DogLog.log("Intake/LeaderRollers/StatorCurrent", super.intakeRollersLeaderStatorCurrent);
    DogLog.log("Intake/LeaderRollers/Velocity", super.intakeRollersLeaderVelocity);
    DogLog.log("Intake/LeaderRollers/Voltage", super.intakeRollersLeaderVoltage);

    // intake rollers 2
    intakeRollersFollowerSim.update(0.02);
    super.intakeRollersFollowerStatorCurrent = intakeRollersFollowerSim.getCurrentDrawAmps();
    super.intakeRollersFollowerVelocity = intakeRollersFollowerSim.getAngularVelocityRPM();
    super.intakeRollersFollowerVoltage = intakeRollersFollowerSim.getInputVoltage();
    DogLog.log("Intake/FollowerRollers/StatorCurrent", super.intakeRollersFollowerStatorCurrent);
    DogLog.log("Intake/FollowerRollers/Velocity", super.intakeRollersFollowerVelocity);
    DogLog.log("Intake/FollowerRollers/Voltage", super.intakeRollersFollowerVoltage);
  }

  @Override
  public void setVoltage(double voltage) {
    intakeRollersLeaderSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
    intakeRollersFollowerSim.setInputVoltage(MathUtil.clamp(-voltage, -12, 12));
  }

  @Override
  public void stop() {
    intakeRollersLeaderSim.setInputVoltage(0);
    intakeRollersFollowerSim.setInputVoltage(0);
  }
}
