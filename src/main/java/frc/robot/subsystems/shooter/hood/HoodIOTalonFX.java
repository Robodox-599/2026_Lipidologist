package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.CANBus;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX extends HoodIO {
    //motors + configuration
    private final TalonFX hoodMotor;
    TalonFXConfiguration hoodConfiguration;
    private final CANBus hoodCANBus;

    //cancoder + configuration
    private final CANcoder hoodCANCoder;


    //motion magic
    private MotionMagicVoltage motionMagic;
    CANcoderConfiguration CANCoderConfig;

    //status signals
    private final StatusSignal<AngularVelocity> hoodVelocityRad;
    private final StatusSignal<Temperature> hoodTemperature;
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodStatorCurrent;
    private final StatusSignal<Current> hoodSupplyCurrent;

    public HoodIOTalonFX() {
        //motors + configuration
        hoodCANBus = new CANBus();
        hoodMotor = new TalonFX(HoodConstants.hoodMotorID, hoodCANBus);
        hoodCANCoder = new CANcoder(HoodConstants.hoodCANCoderID, hoodCANBus);
        hoodConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(HoodConstants.supplyCurrentLimit)
                .withStatorCurrentLimit(HoodConstants.statorCurrentLimit))
            .withSlot0(new Slot0Configs()
                .withKP(HoodConstants.hoodRealkP)
                .withKI(HoodConstants.hoodRealkI)
                .withKD(HoodConstants.hoodRealkD)
                .withKS(HoodConstants.hoodRealkS)
                .withKV(HoodConstants.hoodRealkV))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withFeedbackRemoteSensorID(HoodConstants.hoodCANCoderID)
                .withRotorToSensorRatio(HoodConstants.hoodGearRatio))
            .withClosedLoopGeneral(
                        new ClosedLoopGeneralConfigs()
                                .withContinuousWrap(false))
            .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        CANCoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withMagnetOffset(HoodConstants.hoodMagnetOffset)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withAbsoluteSensorDiscontinuityPoint(HoodConstants.absoluteDiscontinuityPoint));

        //applying configuration
        PhoenixUtil.tryUntilOk(10, () -> hoodMotor.getConfigurator().apply(hoodConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> hoodCANCoder.getConfigurator().apply(CANCoderConfig));


        //status signals
        hoodVelocityRad = hoodMotor.getVelocity();
        hoodTemperature = hoodMotor.getDeviceTemp();
        hoodPosition = hoodCANCoder.getAbsolutePosition();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodStatorCurrent = hoodMotor.getStatorCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

        //Update Frequency
        BaseStatusSignal.setUpdateFrequencyForAll(50, hoodVelocityRad, hoodTemperature, 
        hoodPosition, hoodAppliedVolts, hoodStatorCurrent, hoodSupplyCurrent);

        hoodMotor.optimizeBusUtilization();
        hoodCANCoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(hoodTemperature, 
        hoodPosition, hoodAppliedVolts, hoodStatorCurrent, hoodSupplyCurrent);

        super.positionRotations = hoodPosition.getValueAsDouble();
        super.velocity = hoodVelocityRad.getValueAsDouble();
        super.isHoodInPosition = 
            Math.abs(super.positionRotations - super.targetPosition) < HoodConstants.positionTolerance;

        DogLog.log("Hood/Position", super.positionRotations);
        DogLog.log("Hood/TargetPosition", super.targetPosition);
        DogLog.log("Hood/isHoodAtPosition", super.isHoodInPosition);
    }

    @Override
    public void setPosition(double position) {
        targetPosition = MathUtil.clamp(position, HoodConstants.hoodMinAngleRotations, HoodConstants.hoodMaxAngleRotations);

        super.targetPosition = targetPosition;

        motionMagic = new MotionMagicVoltage(position).withSlot(0).withEnableFOC(true);
        
        hoodMotor.setControl(motionMagic);
    }

 @Override
    public void setVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }
}
