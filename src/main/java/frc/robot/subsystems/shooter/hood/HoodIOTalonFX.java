package frc.robot.subsystems.shooter.hood;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX extends HoodIO {
  private final TalonFX hoodMotor;
  private final CANBus hoodCANBus;
  private final TalonFXConfiguration hoodConfig;

  private final CANcoder hoodCANCoder;
  private CANcoderConfiguration CANCoderConfig;

  private MotionMagicVoltage motionMagic;

  // status signals
  private final StatusSignal<AngularVelocity> hoodVelocityRotsPerSec;
  private final StatusSignal<Temperature> hoodTemperature;
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodStatorCurrent;
  private final StatusSignal<Current> hoodSupplyCurrent;

  public HoodIOTalonFX() {
    hoodCANBus = new CANBus(HoodConstants.hoodCANBus);
    hoodMotor = new TalonFX(HoodConstants.hoodMotorID, hoodCANBus);
    hoodConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(HoodConstants.supplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(HoodConstants.statorCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(HoodConstants.hoodRealkP)
                    .withKI(HoodConstants.hoodRealkI)
                    .withKD(HoodConstants.hoodRealkD)
                    .withKS(HoodConstants.hoodRealkS)
                    .withKV(HoodConstants.hoodRealkV))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(HoodConstants.hoodCANCoderID)
                    .withRotorToSensorRatio(HoodConstants.hoodGearRatio))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(HoodConstants.hoodMaxVelocity)
                    .withMotionMagicAcceleration(HoodConstants.hoodMaxAcceleration));

    // CANCoder
    hoodCANCoder = new CANcoder(HoodConstants.hoodCANCoderID, HoodConstants.hoodCANBus);
    CANCoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withMagnetOffset(HoodConstants.hoodMagnetOffset)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(
                        HoodConstants.absoluteDiscontinuityPoint));

    PhoenixUtil.tryUntilOk(10, () -> hoodCANCoder.getConfigurator().apply(CANCoderConfig, 1));
    PhoenixUtil.tryUntilOk(10, () -> hoodMotor.getConfigurator().apply(hoodConfig, 1));

    motionMagic = new MotionMagicVoltage(targetPositionRots).withSlot(0).withEnableFOC(true);

    // status signal stuff
    hoodVelocityRotsPerSec = hoodMotor.getVelocity();
    hoodTemperature = hoodMotor.getDeviceTemp();
    hoodPosition = hoodCANCoder.getAbsolutePosition();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, hoodVelocityRotsPerSec, hoodPosition, hoodStatorCurrent, hoodSupplyCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(5, hoodTemperature, hoodAppliedVolts);

    hoodMotor.optimizeBusUtilization();
    hoodCANCoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        hoodTemperature, hoodPosition, hoodAppliedVolts, hoodStatorCurrent, hoodSupplyCurrent);

    super.positionRotations = hoodPosition.getValueAsDouble();
    super.RPS = hoodVelocityRotsPerSec.getValueAsDouble();
    super.statorCurrent = hoodStatorCurrent.getValueAsDouble();
    super.supplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    super.temperature = hoodTemperature.getValueAsDouble();
    super.isHoodInPosition =
        Math.abs(super.positionRotations - super.targetPositionRots)
            < HoodConstants.positionTolerance;

    DogLog.log("Hood/Position", super.positionRotations);
    DogLog.log("Hood/TargetPosition", super.targetPositionRots);
    DogLog.log("Hood/StatorCurrent", super.statorCurrent);
    DogLog.log("Hood/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Hood/Temperature", super.temperature);
    DogLog.log("Hood/isHoodAtPosition", super.isHoodInPosition);
  }

  @Override
  public void setHoodPosition(double position) {
    targetPositionRots =
        MathUtil.clamp(
            position, HoodConstants.hoodMinAngleRotations, HoodConstants.hoodMaxAngleRotations);

    super.targetPositionRots = targetPositionRots;

    hoodMotor.setControl(motionMagic.withPosition(position).withEnableFOC(true));
  }

  @Override
  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public void stopHood() {
    hoodMotor.setVoltage(0);
  }
}
