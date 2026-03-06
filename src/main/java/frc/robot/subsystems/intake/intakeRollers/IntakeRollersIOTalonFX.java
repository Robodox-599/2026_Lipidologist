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
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeRollersIOTalonFX extends IntakeRollersIO {
    private final TalonFX intakeRollersMotorOne;
    private final CANBus intakeRollersCanBusOne;
    private final TalonFXConfiguration intakeRollersMotorConfigOne;

    private final TalonFX intakeRollersMotorTwo;
    private final CANBus intakeRollersCanBusTwo;
    private final TalonFXConfiguration intakeRollersMotorConfigTwo;

    private final VoltageOut v_request;

    private final StatusSignal<AngularVelocity> intakeRollersOneVelocity;
    private final StatusSignal<Voltage> intakeRollersOneAppliedVolts;
    private final StatusSignal<Current> intakeRollersOneSupplyCurrent;
    private final StatusSignal<Current> intakeRollersOneStatorCurrent;
    private final StatusSignal<Temperature> intakeRollersOneTemperature;

    private final StatusSignal<AngularVelocity> intakeRollersTwoVelocity;
    private final StatusSignal<Voltage> intakeRollersTwoAppliedVolts;
    private final StatusSignal<Current> intakeRollersTwoSupplyCurrent;
    private final StatusSignal<Current> intakeRollersTwoStatorCurrent;
    private final StatusSignal<Temperature> intakeRollersTwoTemperature;

    public IntakeRollersIOTalonFX() {
        intakeRollersCanBusOne = new CANBus(IntakeRollersConstants.intakeRollersCanBusOne);
        intakeRollersMotorOne = new TalonFX(IntakeRollersConstants.intakeRollersMotorOneID, intakeRollersCanBusOne);

        intakeRollersCanBusTwo = new CANBus(IntakeRollersConstants.intakeRollersCanBusTwo);
        intakeRollersMotorTwo = new TalonFX(IntakeRollersConstants.intkaeRollersMotorTwoID, intakeRollersCanBusTwo);

        intakeRollersMotorConfigOne = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                        .withStatorCurrentLimitEnable(true)
                ).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));
        intakeRollersMotorConfigTwo = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                ).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));
        v_request = new VoltageOut(0);

        PhoenixUtil.tryUntilOk(10, () -> intakeRollersMotorOne.getConfigurator().apply(intakeRollersMotorConfigOne, 1));
        PhoenixUtil.tryUntilOk(10, () -> intakeRollersMotorTwo.getConfigurator().apply(intakeRollersMotorConfigTwo, 1));

        intakeRollersOneVelocity = intakeRollersMotorOne.getVelocity();
        intakeRollersOneAppliedVolts = intakeRollersMotorOne.getMotorVoltage();
        intakeRollersOneSupplyCurrent = intakeRollersMotorOne.getSupplyCurrent();
        intakeRollersOneStatorCurrent = intakeRollersMotorOne.getStatorCurrent();
        intakeRollersOneTemperature = intakeRollersMotorOne.getDeviceTemp();

        intakeRollersTwoVelocity = intakeRollersMotorTwo.getVelocity();
        intakeRollersTwoAppliedVolts = intakeRollersMotorTwo.getMotorVoltage();
        intakeRollersTwoSupplyCurrent = intakeRollersMotorTwo.getSupplyCurrent();
        intakeRollersTwoStatorCurrent = intakeRollersMotorTwo.getStatorCurrent();
        intakeRollersTwoTemperature = intakeRollersMotorTwo.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, 
                //intake rollers 1
                intakeRollersOneVelocity, intakeRollersOneAppliedVolts,
                intakeRollersOneStatorCurrent, intakeRollersOneSupplyCurrent, intakeRollersOneTemperature,
                //intake rollers 2
                intakeRollersTwoVelocity, intakeRollersTwoAppliedVolts, intakeRollersTwoStatorCurrent, 
                intakeRollersTwoSupplyCurrent, intakeRollersTwoTemperature
            );

        intakeRollersMotorOne.optimizeBusUtilization();
        intakeRollersMotorTwo.optimizeBusUtilization();
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(
            //intake rollers 1
            intakeRollersOneVelocity, intakeRollersOneStatorCurrent, intakeRollersOneSupplyCurrent,
            intakeRollersOneAppliedVolts, intakeRollersOneTemperature
            //intake rollers 2
            , intakeRollersTwoVelocity, intakeRollersTwoStatorCurrent, intakeRollersTwoSupplyCurrent,
            intakeRollersTwoAppliedVolts, intakeRollersTwoTemperature
        );

        //intake rollers 1
        super.intakeRollersOneVelocity = intakeRollersOneVelocity.getValueAsDouble();
        super.intakeRollersOneVoltage = intakeRollersOneAppliedVolts.getValueAsDouble();
        super.intakeRollersOneStatorCurrent = intakeRollersOneStatorCurrent.getValueAsDouble();
        super.intakeRollersOneSupplyCurrent = intakeRollersOneSupplyCurrent.getValueAsDouble();
        super.intakeRollersOneTemperature = intakeRollersOneTemperature.getValueAsDouble();

        DogLog.log("Intake/IntakeRollersOne/Velocity", super.intakeRollersOneVelocity);
        DogLog.log("Intake/IntakeRollersOne/Voltage", super.intakeRollersOneVoltage);
        DogLog.log("Intake/IntakeRollersOne/StatorCurrent", super.intakeRollersOneStatorCurrent);
        DogLog.log("Intake/IntakeRollersOne/SupplyCurrent", super.intakeRollersOneSupplyCurrent);
        DogLog.log("Intake/IntakeRollersOne/Temperature", super.intakeRollersOneTemperature);

        //intake rollers 2
        super.intakeRollersTwoVelocity = intakeRollersTwoVelocity.getValueAsDouble();
        super.intakeRollersTwoVoltage = intakeRollersTwoAppliedVolts.getValueAsDouble();
        super.intakeRollersTwoStatorCurrent = intakeRollersTwoStatorCurrent.getValueAsDouble();
        super.intakeRollersTwoSupplyCurrent = intakeRollersTwoSupplyCurrent.getValueAsDouble();
        super.intakeRollersTwoTemperature = intakeRollersTwoTemperature.getValueAsDouble();

        DogLog.log("Intake/IntakeRollersTwo/Velocity", super.intakeRollersTwoVelocity);
        DogLog.log("Intake/IntakeRollersTwo/Voltage", super.intakeRollersOneVoltage);
        DogLog.log("Intake/IntakeRollersTwo/StatorCurrent", super.intakeRollersTwoStatorCurrent);
        DogLog.log("Intake/IntakeRollersTwo/SupplyCurrent", super.intakeRollersTwoSupplyCurrent);
        DogLog.log("Intake/IntakeRollersTwo/Temperature", super.intakeRollersTwoTemperature);
    }

    @Override
    public void stop() {
        intakeRollersMotorOne.setVoltage(0);
        intakeRollersMotorTwo.setVoltage(0);
    }

    @Override
    public void setVoltage(double voltage) {
        intakeRollersMotorOne.setControl(v_request.withOutput(voltage));
        intakeRollersMotorTwo.setControl(v_request.withOutput(voltage));
    }
}
