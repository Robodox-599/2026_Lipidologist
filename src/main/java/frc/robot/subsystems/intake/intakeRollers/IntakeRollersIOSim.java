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
    private final DCMotorSim intakeRollersMotorOneSim;
    private final DCMotorSim intakeRollersMotorTwoSim;

    public IntakeRollersIOSim() {
        intakeRollersMotorOneSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),
                IntakeRollersConstants.rotationalInertia, IntakeRollersConstants.intakeRollersGearRatio),
                DCMotor.getKrakenX60Foc(1));
        intakeRollersMotorTwoSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 
                IntakeRollersConstants.rotationalInertia, IntakeRollersConstants.intakeRollersGearRatio),
                DCMotor.getKrakenX44Foc(1));
    }

    @Override
    public void updateInputs() {
        //intake rollers 1
        intakeRollersMotorOneSim.update(0.02);
        super.intakeRollersLeaderStatorCurrent = intakeRollersMotorOneSim.getCurrentDrawAmps();
        super.intakeRollersLeaderVelocity = intakeRollersMotorOneSim.getAngularVelocityRPM();
        super.intakeRollersLeaderVoltage = intakeRollersMotorOneSim.getInputVoltage();
        DogLog.log("Intake/RollersOne/StatorCurrent", super.intakeRollersLeaderStatorCurrent);
        DogLog.log("Intake/RollersOne/Velocity", super.intakeRollersLeaderVelocity);
        DogLog.log("Intake/RollersOne/Voltage", super.intakeRollersLeaderVoltage);

        //intake rollers 2
        intakeRollersMotorTwoSim.update(0.02);
        super.intakeRollersFollowerStatorCurrent = intakeRollersMotorTwoSim.getCurrentDrawAmps();
        super.intakeRollersFollowerVelocity = intakeRollersMotorTwoSim.getAngularVelocityRPM();
        super.intakeRollersFollowerVoltage = intakeRollersMotorTwoSim.getInputVoltage();
        DogLog.log("Intake/RollersTwo/StatorCurrent", super.intakeRollersFollowerStatorCurrent);
        DogLog.log("Intake/RollersTwo/Velocity", super.intakeRollersFollowerVelocity);
        DogLog.log("Intake/RollersTwo/Voltage", super.intakeRollersFollowerVoltage);

    }

    @Override
    public void setVoltage(double voltage) {
        intakeRollersMotorOneSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
        intakeRollersMotorTwoSim.setInputVoltage(MathUtil.clamp(-voltage, -12, 12));
    }

    @Override
    public void stop(){
        intakeRollersMotorOneSim.setInputVoltage(0);
        intakeRollersMotorTwoSim.setInputVoltage(0);
    }
}