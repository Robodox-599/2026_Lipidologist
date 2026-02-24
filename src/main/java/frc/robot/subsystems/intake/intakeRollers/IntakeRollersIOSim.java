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
    private final DCMotorSim intakeRollersMotorSim;

    public IntakeRollersIOSim() {
        intakeRollersMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),
                IntakeRollersConstants.rotationalInertia, IntakeRollersConstants.intakeRollersGearRatio),
                DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs() {
        intakeRollersMotorSim.update(0.02);
        super.statorCurrent = intakeRollersMotorSim.getCurrentDrawAmps();
        super.velocity = intakeRollersMotorSim.getAngularVelocityRPM();
        super.voltage = intakeRollersMotorSim.getInputVoltage();
        DogLog.log("Intake/Rollers/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Rollers/Velocity", super.velocity);
        DogLog.log("Intake/Rollers/Voltage", super.voltage);

    }

    @Override
    public void setVoltage(double voltage) {
        intakeRollersMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    // @Override
    // public void setVelocity(double velocity) {
    //     intakeRollersMotorSim.setAngularVelocity(velocity);
    // }

    @Override
    public void stop(){
        intakeRollersMotorSim.setInputVoltage(0);
    }
}