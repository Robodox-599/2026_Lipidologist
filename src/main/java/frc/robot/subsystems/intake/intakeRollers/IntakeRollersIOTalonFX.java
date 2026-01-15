// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeRollersIOTalonFX extends IntakeRollersIO {
    private final TalonFX intakeRollersMotor;
    private final TalonFXConfiguration intakeRollersConfig;

    private final StatusSignal<Angle> intakeRollersPosition;
    private final StatusSignal<AngularVelocity> intakeRollersVelocity;
    private final StatusSignal<Voltage> intakeRollersAppliedVolts;
    private final StatusSignal<Current> intakeRollersStatorCurrent;
    private final StatusSignal<Current> intakeRollersSupplyCurrent;
    private final StatusSignal<Temperature> intakeRollersTemperature;

    public IntakeRollersIOTalonFX(){
        intakeRollersMotor = new TalonFX(IntakeRollersConstants.IntakeRollersMotorID, IntakeRollersConstants.IntakeRollersCanBus);
        intakeRollersConfig = new TalonFXConfiguration();

        intakeRollersConfig.Slot0.kP = IntakeRollersConstants.kP;
        intakeRollersConfig.Slot0.kI = IntakeRollersConstants.kI;
        intakeRollersConfig.Slot0.kD = IntakeRollersConstants.kD;
        intakeRollersConfig.Slot0.kS = IntakeRollersConstants.kS;
        intakeRollersConfig.Slot0.kV = IntakeRollersConstants.kV;

        intakeRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeRollersConfig.CurrentLimits.StatorCurrentLimit = IntakeRollersConstants.supplyCurrentLimit;

        intakeRollersPosition = intakeRollersMotor.getPosition(); //?
        intakeRollersVelocity = intakeRollersMotor.getVelocity();
        intakeRollersAppliedVolts = intakeRollersMotor.getMotorVoltage();
        intakeRollersStatorCurrent = intakeRollersMotor.getStatorCurrent();
        intakeRollersSupplyCurrent = intakeRollersMotor.getSupplyCurrent();
        intakeRollersTemperature = intakeRollersMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeRollersPosition, intakeRollersVelocity, intakeRollersAppliedVolts, intakeRollersStatorCurrent, intakeRollersSupplyCurrent, intakeRollersTemperature);
        
        intakeRollersMotor.optimizeBusUtilization();
    }

    public void updateInputs(){
        BaseStatusSignal.refreshAll(intakeRollersPosition, intakeRollersVelocity, intakeRollersAppliedVolts, intakeRollersStatorCurrent, intakeRollersSupplyCurrent, intakeRollersTemperature);

        super.position = intakeRollersPosition.getValueAsDouble();
        super.velocity = intakeRollersVelocity.getValueAsDouble();
        super.voltage = intakeRollersAppliedVolts.getValueAsDouble();
        super.statorCurrent = intakeRollersStatorCurrent.getValueAsDouble();
        super.supplyCurrent = intakeRollersSupplyCurrent.getValueAsDouble();
        super.temperature = intakeRollersTemperature.getValueAsDouble();

        DogLog.log("Intake/Rollers/Position", super.position);
        DogLog.log("Intake/Rollers/Velocity", super.velocity);
        DogLog.log("Intake/Rollers/Voltage", super.voltage);
        DogLog.log("Intake/Rollers/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Rollers/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Intake/Rollers/Temperature", super.temperature);
    }

    @Override
    public void stop(){
        intakeRollersMotor.stopMotor();
    }

    @Override
    public void setVelocity(double velocity){
        intakeRollersMotor.set(velocity);
    }


}
