// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants;

/** Add your docs here. */
public class IntakeWristIOSim extends IntakeWristIO {
    private final DCMotorSim intakeWristMotorSim;
    private final ProfiledPIDController pid; // type is trapazoidal motion

    public IntakeWristIOSim() {
        // intakeWristMotorSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), IntakeWristConstants.gearRatio, 0.01,
        //         IntakeWristConstants.wristLengthMeters, IntakeWristConstants.minAngleRad,
        //         IntakeWristConstants.maxAngleRad, true, Units.degreesToRadians(IntakeWristConstants.startAngleRad));
        intakeWristMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),
                IntakeWristConstants.rotationalInertia, IntakeWristConstants.gearRatio),
                DCMotor.getKrakenX60Foc(1));
        pid = new ProfiledPIDController(IntakeWristConstants.kPSim, IntakeWristConstants.kISim,
                IntakeWristConstants.kDSim,
                new Constraints(IntakeWristConstants.maxVelocitySim, IntakeWristConstants.maxAccelerationSim));
        // ff = new ArmFeedforward(IntakeWristConstants.kSSim,
        // IntakeWristConstants.kGSim, IntakeWristConstants.kVSim);
    }

    @Override
    public void updateInputs() {
        intakeWristMotorSim.update(0.02);
        super.statorCurrent = intakeWristMotorSim.getCurrentDrawAmps();
        super.currentPosition = intakeWristMotorSim.getAngularPositionRad();
        super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;

        DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Wrist/Position", super.currentPosition);
        DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
        DogLog.log("Intake/Wrist/Voltage", super.voltage);
        DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    @Override
    public void setPosition(double position) {
        super.targetPosition = position;
        setVoltage(pid.calculate(super.currentPosition, super.targetPosition));
    }

    @Override
    public void setVoltage(double voltage) {
        super.voltage = voltage;
        intakeWristMotorSim.setInputVoltage(voltage);
    }
}