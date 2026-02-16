// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeRollersIOTalonFX extends IntakeRollersIO {
    private final TalonFX intakeRollersMotor;
    private final CANBus intakeRollersCanBus;
    private final TalonFXConfiguration intakeRollersMotorConfig;

    private final StatusSignal<AngularVelocity> intakeRollersVelocity;
    private final StatusSignal<Voltage> intakeRollersAppliedVolts;
    private final StatusSignal<Current> intakeRollersSupplyCurrent;
    private final StatusSignal<Current> intakeRollersStatorCurrent;
    private final StatusSignal<Temperature> intakeRollersTemperature;

    public IntakeRollersIOTalonFX() {
        intakeRollersCanBus = new CANBus(IntakeRollersConstants.intakeRollersCanBus);
        intakeRollersMotor = new TalonFX(IntakeRollersConstants.intakeRollersMotorID, intakeRollersCanBus);
        intakeRollersMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                ).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        // intakeRollersConfig.Slot0.kP = IntakeRollersConstants.kP;
        // intakeRollersConfig.Slot0.kI = IntakeRollersConstants.kI;
        // intakeRollersConfig.Slot0.kD = IntakeRollersConstants.kD;
        // intakeRollersConfig.Slot0.kS = IntakeRollersConstants.kS;
        // intakeRollersConfig.Slot0.kV = IntakeRollersConstants.kV;

        PhoenixUtil.tryUntilOk(10, () -> intakeRollersMotor.getConfigurator().apply(intakeRollersMotorConfig, 1));

        intakeRollersVelocity = intakeRollersMotor.getVelocity();
        intakeRollersAppliedVolts = intakeRollersMotor.getMotorVoltage();
        intakeRollersSupplyCurrent = intakeRollersMotor.getSupplyCurrent();
        intakeRollersStatorCurrent = intakeRollersMotor.getStatorCurrent();
        intakeRollersTemperature = intakeRollersMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeRollersVelocity, intakeRollersAppliedVolts,
                intakeRollersStatorCurrent, intakeRollersSupplyCurrent, intakeRollersTemperature);

        intakeRollersMotor.optimizeBusUtilization();
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(intakeRollersVelocity, intakeRollersStatorCurrent, intakeRollersSupplyCurrent,
                intakeRollersAppliedVolts, intakeRollersTemperature);

        super.velocity = intakeRollersVelocity.getValueAsDouble();
        super.voltage = intakeRollersAppliedVolts.getValueAsDouble();
        super.statorCurrent = intakeRollersStatorCurrent.getValueAsDouble();
        super.supplyCurrent = intakeRollersSupplyCurrent.getValueAsDouble();
        super.temperature = intakeRollersTemperature.getValueAsDouble();

        DogLog.log("Intake/Rollers/Velocity", super.velocity);
        DogLog.log("Intake/Rollers/Voltage", super.voltage);
        DogLog.log("Intake/Rollers/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Rollers/SupplyCurrent", supplyCurrent);
        DogLog.log("Intake/Rollers/Temperature", super.temperature);
    }

    @Override
    public void stop() {
        intakeRollersMotor.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        intakeRollersMotor.setControl(new VoltageOut(voltage));
    }
}
