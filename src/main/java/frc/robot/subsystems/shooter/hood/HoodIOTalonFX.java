package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Motor.TalonFXWrapper;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX extends HoodIO {

  private final TalonFXWrapper hoodMotorWrapper;
  private final TalonFX hoodMotor;

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
    hoodMotorWrapper = new TalonFXWrapper(HoodConstants.hoodMotor);
    hoodMotor = hoodMotorWrapper.getTalonFX();

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

    motionMagic = new MotionMagicVoltage(targetPositionRots).withSlot(0).withEnableFOC(true);

    // status signal stuff
    hoodVelocityRotsPerSec = hoodMotor.getVelocity();
    hoodTemperature = hoodMotor.getDeviceTemp();
    hoodPosition = hoodCANCoder.getAbsolutePosition();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodStatorCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        hoodVelocityRotsPerSec,
        hoodTemperature,
        hoodPosition,
        hoodAppliedVolts,
        hoodStatorCurrent,
        hoodSupplyCurrent);

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
  public void setPosition(double position) {
    targetPositionRots =
        MathUtil.clamp(
            position, HoodConstants.hoodMinAngleRotations, HoodConstants.hoodMaxAngleRotations);

    super.targetPositionRots = targetPositionRots;

    hoodMotor.setControl(motionMagic.withPosition(position).withEnableFOC(true));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotorWrapper.setVoltage(voltage);
  }

  @Override
  public void stop() {
    hoodMotorWrapper.stop();
  }
}
