// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

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
    private MotionMagicVoltage m_request;

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

        canCoderConfig.MagnetSensor.MagnetOffset = IntakeWristConstants.magnetOffset;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeWristConstants.absoluteDiscontinuityPoint;

        intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeWristConfig.CurrentLimits.SupplyCurrentLimit = IntakeWristConstants.supplyCurrentLimit; //check needed

        intakeWristConfig.Slot0.kP = IntakeWristConstants.kP;
        intakeWristConfig.Slot0.kI = IntakeWristConstants.kI;
        intakeWristConfig.Slot0.kD = IntakeWristConstants.kD;
        intakeWristConfig.Slot0.kS = IntakeWristConstants.kS;
        intakeWristConfig.Slot0.kV = IntakeWristConstants.kV;
        intakeWristConfig.Slot0.kG = IntakeWristConstants.kG;

        intakeWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        intakeWristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakeWristConfig.Feedback.FeedbackRemoteSensorID = IntakeWristConstants.intakeWristCanCoderID;
        intakeWristConfig.Feedback.RotorToSensorRatio = IntakeWristConstants.gearRatio;
        intakeWristConfig.ClosedLoopGeneral.ContinuousWrap = false;

        intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeWristConstants.maxVelocity;
        intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeWristConstants.maxAcceleration;

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
        super.currentPosition = intakeWristPosition.getValueAsDouble();
        super.velocity = intakeWristVelocity.getValueAsDouble();
        super.voltage = intakeWristAppliedVolts.getValueAsDouble();
        super.statorCurrent = intakeWristStatorCurrent.getValueAsDouble();
        super.supplyCurrent = intakeWristSupplyCurrent.getValueAsDouble();
        super.temperature = intakeWristTemperature.getValueAsDouble();

        super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;

        DogLog.log("Intake/Wrist/Position", super.currentPosition);
        DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
        DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
        DogLog.log("Intake/Wrist/Velocity", super.velocity);
        DogLog.log("Intake/Wrist/Voltage", super.voltage);
        DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
        DogLog.log("Intake/Wrist/SupplyCurrent", super.supplyCurrent);
        DogLog.log("Intake/Wrist/Temperature", super.temperature);  
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
