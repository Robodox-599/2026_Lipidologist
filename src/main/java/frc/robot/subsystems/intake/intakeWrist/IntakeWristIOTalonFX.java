// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeWristIOTalonFX extends IntakeWristIO{
    public final TalonFX intakeWristMotor;
    public final CANcoder intakeWristCanCoder;
    public final TalonFXConfiguration intakeWristConfig;
    public final CANcoderConfiguration canCoderConfig;
    // make sure to make this private (doesn't affect the functionality, but makes the code style consistant)
    MotionMagicVoltage m_request;

    public StatusSignal<Angle> intakeWristPosition;
    public StatusSignal<AngularVelocity> intakeWristVelocity;
    public StatusSignal<Voltage> intakeWristAppliedVolts;
    public StatusSignal<Current> intakeWristStatorCurrent;
    public StatusSignal<Current> intakeWristSupplyCurrent;
    public StatusSignal<Temperature> intakeWristTemperature;
    
    public IntakeWristIOTalonFX(){
        intakeWristMotor = new TalonFX(IntakeWristConstants.intakeWristMotorID, IntakeWristConstants.intakeWristCanBus);
        intakeWristCanCoder = new CANcoder(IntakeWristConstants.intakeWristCanCoderID);
        intakeWristConfig = new TalonFXConfiguration();
        canCoderConfig = new CANcoderConfiguration();
        m_request = new MotionMagicVoltage(0);

        canCoderConfig.MagnetSensor.MagnetOffset = IntakeWristConstants.intakeWristMagnetOffset;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeWristConstants.intakeAbsoluteDiscontinuityPoint;

        intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeWristConfig.CurrentLimits.StatorCurrentLimit = IntakeWristConstants.supplyCurrentLimit;

        intakeWristConfig.Slot0.kP = IntakeWristConstants.kP;
        intakeWristConfig.Slot0.kI = IntakeWristConstants.kI;
        intakeWristConfig.Slot0.kD = IntakeWristConstants.kD;
        intakeWristConfig.Slot0.kS = IntakeWristConstants.kS;
        intakeWristConfig.Slot0.kV = IntakeWristConstants.kV;
        intakeWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        intakeWristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakeWristConfig.Feedback.FeedbackRemoteSensorID = IntakeWristConstants.intakeWristCanCoderID;
        intakeWristConfig.Feedback.RotorToSensorRatio = IntakeWristConstants.intakeWristGearRatio;
        intakeWristConfig.ClosedLoopGeneral.ContinuousWrap = false;

        intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeWristConstants.intakeWristMaxVelocity;
        intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeWristConstants.intakeWristMaxAcceleration;

        PhoenixUtil.tryUntilOk(10, () -> intakeWristMotor.getConfigurator().apply(intakeWristConfig));
        PhoenixUtil.tryUntilOk(10, () -> intakeWristCanCoder.getConfigurator().apply(canCoderConfig));

        intakeWristMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeWristPosition = intakeWristMotor.getPosition();
        intakeWristVelocity = intakeWristMotor.getVelocity();
        intakeWristAppliedVolts = intakeWristMotor.getMotorVoltage();
        intakeWristStatorCurrent = intakeWristMotor.getStatorCurrent();
        intakeWristSupplyCurrent = intakeWristMotor.getSupplyCurrent();
        intakeWristTemperature = intakeWristMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeWristPosition, intakeWristVelocity, intakeWristAppliedVolts, intakeWristStatorCurrent, intakeWristSupplyCurrent, intakeWristTemperature);

        intakeWristMotor.optimizeBusUtilization();        
        intakeWristCanCoder.optimizeBusUtilization();
    }

    public void updateInputs(){
        super.targetPosition = intakeWristPosition.getValueAsDouble();
        super.velocity = intakeWristPosition.getValueAsDouble();
        super.voltage = intakeWristAppliedVolts.getValueAsDouble();
        super.statorCurrent = intakeWristStatorCurrent.getValueAsDouble();
        super.supplyCurrent = intakeWristSupplyCurrent.getValueAsDouble();
        super.temperature = intakeWristTemperature.getValueAsDouble();

        super.atSetpoint = Math.abs(super.targetPosition - super.wantedPosition) < 0.02;

        DogLog.log("Intake/Wrist/position", targetPosition);
        DogLog.log("Intake/Wrist/IsWristInPosition", super.atSetpoint);
        DogLog.log("Intake/Wrist/Velocity", velocity);
        DogLog.log("Intake/Wrist/Voltage", voltage);
        DogLog.log("Intake/Wrist/StatorCurrent", statorCurrent);
        DogLog.log("Intake/Wrist/SupplyCurrent", supplyCurrent);
        DogLog.log("Intake/Wrist/Temperature", temperature);  
    }

    /**
         * stops the movement of the intakeWristMotor
         * <ul>
         *  <li> Units: rotations
         * </ui>
         */
    @Override
    public void stop(){
        intakeWristMotor.stopMotor();
    }

    @Override
    public void setPosition(double position){
        super.targetPosition = position;
        intakeWristMotor.setControl(m_request.withPosition(position));
    }

    @Override
    public double getPosition(){
        return intakeWristMotor.getPosition().getValueAsDouble();
    }
}
