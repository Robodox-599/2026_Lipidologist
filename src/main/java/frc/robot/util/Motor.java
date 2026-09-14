package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Motor {

  public interface RobotMotor {
    void setVoltage(double volts);

    void setPositionRotations(double targetRotations);

    double getVelocityRPS();

    double getPositionRotations();

    void stop();
  }

  public record MotorConstants(
      String name,
      int motorID,
      String canBus,
      InvertedValue invert,
      NeutralModeValue neutralMode,
      double gearRatio,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      Double forwardSoftLimitRotations,
      Double reverseSoftLimitRotations,
      double openLoopRampRateSec,
      double closedLoopRampRateSec,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kG,
      Double mmCruiseVelocity,
      Double mmAcceleration,
      FeedbackSensorSourceValue feedbackSource,
      Integer remoteSensorID,
      Double rotorToSensorRatio) {
    public MotorConstants(
        String name,
        int motorID,
        String canBus,
        InvertedValue invert,
        NeutralModeValue neutralMode,
        double gearRatio,
        double statorCurrentLimitAmps,
        double supplyCurrentLimitAmps,
        Double forwardSoftLimitRotations,
        Double reverseSoftLimitRotations,
        double openLoopRampRateSec,
        double closedLoopRampRateSec,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kG) {
      this(
          name,
          motorID,
          canBus,
          invert,
          neutralMode,
          gearRatio,
          statorCurrentLimitAmps,
          supplyCurrentLimitAmps,
          forwardSoftLimitRotations,
          reverseSoftLimitRotations,
          openLoopRampRateSec,
          closedLoopRampRateSec,
          kP,
          kI,
          kD,
          kS,
          kV,
          kG,
          null,
          null,
          null,
          null,
          null);
    }
  }

  public static class TalonFXWrapper implements RobotMotor {
    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private final double gearRatio;
    private final boolean hasMotionMagic;

    public TalonFXWrapper(MotorConstants constants) {
      this.gearRatio = constants.gearRatio();
      this.motor = new TalonFX(constants.motorID(), constants.canBus());
      this.hasMotionMagic = (constants.mmCruiseVelocity() != null);

      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.Inverted = constants.invert();
      config.MotorOutput.NeutralMode = constants.neutralMode();
      config.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimitAmps();
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimitAmps();
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      config.Slot0.kP = constants.kP();
      config.Slot0.kI = constants.kI();
      config.Slot0.kD = constants.kD();
      config.Slot0.kS = constants.kS();
      config.Slot0.kV = constants.kV();
      config.Slot0.kG = constants.kG();

      if (constants.forwardSoftLimitRotations() != null) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            constants.forwardSoftLimitRotations() * gearRatio;
      }
      if (constants.reverseSoftLimitRotations() != null) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            constants.reverseSoftLimitRotations() * gearRatio;
      }

      if (hasMotionMagic) {
        config.MotionMagic.MotionMagicCruiseVelocity = constants.mmCruiseVelocity();
        if (constants.mmAcceleration() != null) {
          config.MotionMagic.MotionMagicAcceleration = constants.mmAcceleration();
        }
      }

      if (constants.feedbackSource() != null) {
        config.Feedback.FeedbackSensorSource = constants.feedbackSource();
        if (constants.remoteSensorID() != null) {
          config.Feedback.FeedbackRemoteSensorID = constants.remoteSensorID();
        }
        if (constants.rotorToSensorRatio() != null) {
          config.Feedback.RotorToSensorRatio = constants.rotorToSensorRatio();
        }
      }

      motor.getConfigurator().apply(config);
    }

    public TalonFX getTalonFX() {
      return motor;
    }

    @Override
    public void setVoltage(double volts) {
      motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setPositionRotations(double targetRotations) {
      double motorRotations = targetRotations * gearRatio;
      if (hasMotionMagic) {
        motor.setControl(motionMagicRequest.withPosition(motorRotations));
      } else {
        motor.setControl(positionRequest.withPosition(motorRotations));
      }
    }

    @Override
    public double getVelocityRPS() {
      return motor.getVelocity().getValueAsDouble() / gearRatio;
    }

    @Override
    public double getPositionRotations() {
      return motor.getPosition().getValueAsDouble() / gearRatio;
    }

    @Override
    public void stop() {
      motor.stopMotor();
    }
  }
}
