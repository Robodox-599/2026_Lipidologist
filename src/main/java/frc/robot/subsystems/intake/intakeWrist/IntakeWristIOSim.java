// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeWristIOSim extends IntakeWristIO {
  private final DCMotorSim intakeWristMotorSim;
  private final ProfiledPIDController pid;

  public IntakeWristIOSim() {
    intakeWristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                IntakeWristConstants.rotationalInertia,
          IntakeWristConstants.intakeWristMotor.gearRatio()),
            DCMotor.getKrakenX60Foc(1));
    pid =
        new ProfiledPIDController(
            IntakeWristConstants.kPSim,
            IntakeWristConstants.kISim,
            IntakeWristConstants.kDSim,
            new Constraints(
                IntakeWristConstants.maxVelocitySim, IntakeWristConstants.maxAccelerationSim));
  }

  @Override
  public void updateInputs() {
    intakeWristMotorSim.update(0.02);
    super.currentPosition = intakeWristMotorSim.getAngularPositionRad() / (2.0 * Math.PI);
    super.voltage = intakeWristMotorSim.getInputVoltage();
    super.statorCurrent = intakeWristMotorSim.getCurrentDrawAmps();
    super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;

    DogLog.log("Intake/Wrist/Position", super.currentPosition);
    DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
    DogLog.log("Intake/Wrist/Voltage", super.voltage);
    DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
    DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
  }

  @Override
  public void stopIntakeWrist() {
    setIntakeWristVoltage(0);
  }

  @Override
  public void setIntakeWristPosition(double position) {
    super.targetPosition = position;
    setIntakeWristVoltage(pid.calculate(super.currentPosition, super.targetPosition));
  }

  @Override
  public double getIntakeWristPosition() {
    return super.currentPosition;
  }

  @Override
  public void setIntakeWristVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    super.voltage = clampedVoltage;
    intakeWristMotorSim.setInputVoltage(clampedVoltage);
  }
}
