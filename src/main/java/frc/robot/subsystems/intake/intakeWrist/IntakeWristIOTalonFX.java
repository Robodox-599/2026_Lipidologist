// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeWristIOTalonFX extends IntakeWristIO {
    public final TalonFX intakeWristMotor;
    public final CANcoder intakeWristCanCoder;
    public final TalonFXConfiguration intakeWristConfig;
    public final CANcoderConfiguration canCoderConfig;
    public final CANBus intakeWristCanBus;
    public final CANBus intakeWristCancoderCanBus;
    private MotionMagicVoltage m_request;

    public final StatusSignal<Angle> intakeWristPosition;
    public final StatusSignal<Voltage> intakeWristAppliedVolts;
    public final StatusSignal<Current> intakeWristStatorCurrent;
    public final StatusSignal<Current> intakeWristSupplyCurrent;
    public final StatusSignal<Temperature> intakeWristTemperature;

    public IntakeWristIOTalonFX() {
        intakeWristCanBus = new CANBus(IntakeWristConstants.intakeWristCanBus);
        intakeWristMotor = new TalonFX(IntakeWristConstants.intakeWristMotorID, intakeWristCanBus);
        intakeWristCancoderCanBus = new CANBus(IntakeWristConstants.intakeWristCanBus);
        intakeWristCanCoder = new CANcoder(IntakeWristConstants.intakeWristCanCoderID, intakeWristCancoderCanBus);
        intakeWristConfig = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(IntakeWristConstants.supplyCurrentLimit)
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(IntakeWristConstants.statorCurrentLimit))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(IntakeWristConstants.kP)
                                .withKI(IntakeWristConstants.kI)
                                .withKD(IntakeWristConstants.kD)
                                .withKV(IntakeWristConstants.kV)
                                .withKS(IntakeWristConstants.kS)
                                .withKG(IntakeWristConstants.kG)
                                .withGravityType(GravityTypeValue.Arm_Cosine))
                .withFeedback(
                        new FeedbackConfigs()
                                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                                .withFeedbackRemoteSensorID(IntakeWristConstants.intakeWristCanCoderID)
                                .withRotorToSensorRatio(IntakeWristConstants.gearRatio))
                .withClosedLoopGeneral(
                        new ClosedLoopGeneralConfigs()
                                .withContinuousWrap(false))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(IntakeWristConstants.maxVelocity)
                                .withMotionMagicAcceleration(IntakeWristConstants.maxAcceleration))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
        canCoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(
                        new MagnetSensorConfigs()
                                .withMagnetOffset(IntakeWristConstants.magnetOffset)
                                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                                .withAbsoluteSensorDiscontinuityPoint(IntakeWristConstants.absoluteDiscontinuityPoint)

                );
        m_request = new MotionMagicVoltage(0);

        PhoenixUtil.tryUntilOk(10, () -> intakeWristMotor.getConfigurator().apply(intakeWristConfig));
        PhoenixUtil.tryUntilOk(10, () -> intakeWristCanCoder.getConfigurator().apply(canCoderConfig));

        // intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeWristPosition = intakeWristCanCoder.getAbsolutePosition();
        intakeWristAppliedVolts = intakeWristMotor.getMotorVoltage();
        intakeWristStatorCurrent = intakeWristMotor.getStatorCurrent();
        intakeWristSupplyCurrent = intakeWristMotor.getSupplyCurrent();
        intakeWristTemperature = intakeWristMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeWristPosition, intakeWristAppliedVolts,
                intakeWristStatorCurrent, intakeWristSupplyCurrent, intakeWristTemperature);

        intakeWristMotor.optimizeBusUtilization();
        intakeWristCanCoder.optimizeBusUtilization();
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(intakeWristPosition, intakeWristAppliedVolts, intakeWristStatorCurrent,
                intakeWristSupplyCurrent, intakeWristTemperature);

        super.currentPosition = intakeWristPosition.getValueAsDouble();
        super.voltage = intakeWristAppliedVolts.getValueAsDouble();
        super.statorCurrent = intakeWristStatorCurrent.getValueAsDouble();
        super.supplyCurrent = intakeWristSupplyCurrent.getValueAsDouble();
        super.temperature = intakeWristTemperature.getValueAsDouble();

        super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;

        DogLog.log("Intake/Wrist/Position", super.currentPosition);
        DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
        DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
        DogLog.log("Intake/Wrist/Voltage", super.voltage);
        DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Wrist/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Intake/Wrist/Temperature", super.temperature);
    }

    /**
     * stops the movement of the intakeWristMotor
     * <ul>
     * <li>Units: rotations
     * </ui>
     */
    @Override
    public void stop() {
        intakeWristMotor.stopMotor();
    }

    @Override
    public void setPosition(double position) {
        super.targetPosition = position;
        intakeWristMotor.setControl(m_request.withPosition(position));
    }

    @Override
    public double getPosition() {
        return intakeWristMotor.getPosition().getValueAsDouble();
    }
}
