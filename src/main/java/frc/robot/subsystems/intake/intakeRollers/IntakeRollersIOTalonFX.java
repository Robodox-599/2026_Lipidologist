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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeRollersIOTalonFX extends IntakeRollersIO {
  private final TalonFX intakeRollersLeaderMotor;
  private final CANBus intakeRollersLeaderCanBus;
  private final TalonFXConfiguration intakeRollersLeaderConfig;

  private final TalonFX intakeRollersFollowerMotor;
  private final CANBus intakeRollersFollowerCanBus;
  private final TalonFXConfiguration intakeRollersFollowerConfig;

  private final VoltageOut v_leader_request;
  private final VoltageOut v_follower_request;

  private final StatusSignal<AngularVelocity> intakeRollersLeaderVelocity;
  private final StatusSignal<Voltage> intakeRollersLeaderAppliedVolts;
  private final StatusSignal<Current> intakeRollersLeaderSupplyCurrent;
  private final StatusSignal<Current> intakeRollersLeaderStatorCurrent;
  private final StatusSignal<Temperature> intakeRollersLeaderTemperature;

  private final StatusSignal<AngularVelocity> intakeRollersFollowerVelocity;
  private final StatusSignal<Voltage> intakeRollersFollowerAppliedVolts;
  private final StatusSignal<Current> intakeRollersFollowerSupplyCurrent;
  private final StatusSignal<Current> intakeRollersFollowerStatorCurrent;
  private final StatusSignal<Temperature> intakeRollersFollowerTemperature;

  public IntakeRollersIOTalonFX() {
    intakeRollersLeaderCanBus = new CANBus(IntakeRollersConstants.intakeRollersLeaderCanBus);
    intakeRollersLeaderMotor =
        new TalonFX(IntakeRollersConstants.intakeRollersLeaderMotorID, intakeRollersLeaderCanBus);

    intakeRollersFollowerCanBus = new CANBus(IntakeRollersConstants.intakeRollersFollowerCanBus);
    intakeRollersFollowerMotor =
        new TalonFX(
            IntakeRollersConstants.intakeRollersFollowerMotorID, intakeRollersFollowerCanBus);

    intakeRollersLeaderConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
    // .withOpenLoopRamps(new
    // OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1.0))
    ;
    v_leader_request = new VoltageOut(0);
    v_follower_request = new VoltageOut(0);

    intakeRollersFollowerConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(IntakeRollersConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeRollersConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
    // .withOpenLoopRamps(new
    // OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1.0))
    ;
    PhoenixUtil.tryUntilOk(
        10, () -> intakeRollersLeaderMotor.getConfigurator().apply(intakeRollersLeaderConfig, 1));
    PhoenixUtil.tryUntilOk(
        10,
        () -> intakeRollersFollowerMotor.getConfigurator().apply(intakeRollersFollowerConfig, 1));

    intakeRollersLeaderVelocity = intakeRollersLeaderMotor.getVelocity();
    intakeRollersLeaderAppliedVolts = intakeRollersLeaderMotor.getMotorVoltage();
    intakeRollersLeaderSupplyCurrent = intakeRollersLeaderMotor.getSupplyCurrent();
    intakeRollersLeaderStatorCurrent = intakeRollersLeaderMotor.getStatorCurrent();
    intakeRollersLeaderTemperature = intakeRollersLeaderMotor.getDeviceTemp();

    intakeRollersFollowerVelocity = intakeRollersFollowerMotor.getVelocity();
    intakeRollersFollowerAppliedVolts = intakeRollersFollowerMotor.getMotorVoltage();
    intakeRollersFollowerSupplyCurrent = intakeRollersFollowerMotor.getSupplyCurrent();
    intakeRollersFollowerStatorCurrent = intakeRollersFollowerMotor.getStatorCurrent();
    intakeRollersFollowerTemperature = intakeRollersFollowerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        // intake rollers 1
        intakeRollersLeaderVelocity,
        intakeRollersLeaderAppliedVolts,
        intakeRollersLeaderStatorCurrent,
        intakeRollersLeaderSupplyCurrent,
        intakeRollersLeaderTemperature,
        // intake rollers 2
        intakeRollersFollowerVelocity,
        intakeRollersFollowerAppliedVolts,
        intakeRollersFollowerStatorCurrent,
        intakeRollersFollowerSupplyCurrent,
        intakeRollersFollowerTemperature);

    intakeRollersLeaderMotor.optimizeBusUtilization();
    intakeRollersFollowerMotor.optimizeBusUtilization();
  }

  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        // intake rollers 1
        intakeRollersLeaderVelocity,
        intakeRollersLeaderStatorCurrent,
        intakeRollersLeaderSupplyCurrent,
        intakeRollersLeaderAppliedVolts,
        intakeRollersLeaderTemperature
        // intake rollers 2
        ,
        intakeRollersFollowerVelocity,
        intakeRollersFollowerStatorCurrent,
        intakeRollersFollowerSupplyCurrent,
        intakeRollersFollowerAppliedVolts,
        intakeRollersFollowerTemperature);

    // intake rollers 1
    super.intakeRollersLeaderVelocity = intakeRollersLeaderVelocity.getValueAsDouble();
    super.intakeRollersLeaderVoltage = intakeRollersLeaderAppliedVolts.getValueAsDouble();
    super.intakeRollersLeaderStatorCurrent = intakeRollersLeaderStatorCurrent.getValueAsDouble();
    super.intakeRollersLeaderSupplyCurrent = intakeRollersLeaderSupplyCurrent.getValueAsDouble();
    super.intakeRollersLeaderTemperature = intakeRollersLeaderTemperature.getValueAsDouble();

    DogLog.log("Intake/LeaderRollers/Velocity", super.intakeRollersLeaderVelocity);
    DogLog.log("Intake/LeaderRollers/Voltage", super.intakeRollersLeaderVoltage);
    DogLog.log("Intake/LeaderRollers/StatorCurrent", super.intakeRollersLeaderStatorCurrent);
    DogLog.log("Intake/LeaderRollers/SupplyCurrent", super.intakeRollersLeaderSupplyCurrent);
    DogLog.log("Intake/LeaderRollers/Temperature", super.intakeRollersLeaderTemperature);

    // intake rollers 2
    super.intakeRollersFollowerVelocity = intakeRollersFollowerVelocity.getValueAsDouble();
    super.intakeRollersFollowerVoltage = intakeRollersFollowerAppliedVolts.getValueAsDouble();
    super.intakeRollersFollowerStatorCurrent =
        intakeRollersFollowerStatorCurrent.getValueAsDouble();
    super.intakeRollersFollowerSupplyCurrent =
        intakeRollersFollowerSupplyCurrent.getValueAsDouble();
    super.intakeRollersFollowerTemperature = intakeRollersFollowerTemperature.getValueAsDouble();

    DogLog.log("Intake/FollowerRollers/Velocity", super.intakeRollersFollowerVelocity);
    DogLog.log("Intake/FollowerRollers/Voltage", super.intakeRollersLeaderVoltage);
    DogLog.log("Intake/FollowerRollers/StatorCurrent", super.intakeRollersFollowerStatorCurrent);
    DogLog.log("Intake/FollowerRollers/SupplyCurrent", super.intakeRollersFollowerSupplyCurrent);
    DogLog.log("Intake/FollowerRollers/Temperature", super.intakeRollersFollowerTemperature);
  }

  @Override
  public void stop() {
    intakeRollersLeaderMotor.setVoltage(0);
    intakeRollersFollowerMotor.setVoltage(0);
  }

  @Override
  public void setVoltage(double voltage) {
    intakeRollersLeaderMotor.setControl(v_leader_request.withOutput(voltage).withEnableFOC(true));
    intakeRollersFollowerMotor.setControl(
        v_follower_request.withOutput(voltage).withEnableFOC(true));
  }
}
