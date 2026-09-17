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
								DCMotor.getKrakenX44Foc(1),
								IntakeRollersConstants.rotationalInertia,
								IntakeRollersConstants.intakeRollersLeader.gearRatio()),
						DCMotor.getKrakenX44Foc(1));
		intakeRollersFollowerSim =
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(
								DCMotor.getKrakenX44Foc(1),
								IntakeRollersConstants.rotationalInertia,
								IntakeRollersConstants.intakeRollersFollower.gearRatio()),
						DCMotor.getKrakenX44Foc(1));
	}

	@Override
	public void updateInputs() {
		intakeRollersLeaderSim.update(0.02);
		super.intakeRollersLeaderVelocity = intakeRollersLeaderSim.getAngularVelocityRPM() / 60.0; // converts to rotations per second
		super.intakeRollersLeaderVoltage = intakeRollersLeaderSim.getInputVoltage();
		super.intakeRollersLeaderStatorCurrent = intakeRollersLeaderSim.getCurrentDrawAmps();

		DogLog.log("Intake/LeaderRollers/Velocity", super.intakeRollersLeaderVelocity);
		DogLog.log("Intake/LeaderRollers/Voltage", super.intakeRollersLeaderVoltage);
		DogLog.log("Intake/LeaderRollers/StatorCurrent", super.intakeRollersLeaderStatorCurrent);

		intakeRollersFollowerSim.update(0.02);
		super.intakeRollersFollowerVelocity = intakeRollersFollowerSim.getAngularVelocityRPM() / 60.0; // converts to rotations per second
		super.intakeRollersFollowerVoltage = intakeRollersFollowerSim.getInputVoltage();
		super.intakeRollersFollowerStatorCurrent = intakeRollersFollowerSim.getCurrentDrawAmps();

		DogLog.log("Intake/FollowerRollers/Velocity", super.intakeRollersFollowerVelocity);
		DogLog.log("Intake/FollowerRollers/Voltage", super.intakeRollersFollowerVoltage);
		DogLog.log("Intake/FollowerRollers/StatorCurrent", super.intakeRollersFollowerStatorCurrent);
	}

	@Override
	public void setIntakeRollersVoltage(double voltage) {
		double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
		intakeRollersLeaderSim.setInputVoltage(clampedVoltage);
		intakeRollersFollowerSim.setInputVoltage(clampedVoltage);
	}

	@Override
	public void stopIntakeRollers() {
		intakeRollersLeaderSim.setInputVoltage(0.0);
		intakeRollersFollowerSim.setInputVoltage(0.0);
	}
}
