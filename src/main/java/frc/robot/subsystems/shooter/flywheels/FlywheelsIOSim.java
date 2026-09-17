package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelsIOSim extends FlywheelsIO {
  private static final double loopPeriodSeconds = 0.02;

  private final DCMotorSim flywheelMotorSim;
  private final PIDController velocityController;
  private final SimpleMotorFeedforward feedforward;

  public FlywheelsIOSim() {
    flywheelMotorSim = createMotorSim(FlywheelsConstants.flywheelLeader.gearRatio());

    velocityController =
        new PIDController(
            FlywheelsConstants.flywheelLeader.kP(),
            FlywheelsConstants.flywheelLeader.kI(),
            FlywheelsConstants.flywheelLeader.kD());
    feedforward =
        new SimpleMotorFeedforward(
            FlywheelsConstants.flywheelLeader.kS(), FlywheelsConstants.flywheelLeader.kV());
  }

  private static DCMotorSim createMotorSim(double gearRatio) {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), FlywheelsConstants.flywheelMOI, gearRatio),
        DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs() {
    flywheelMotorSim.update(loopPeriodSeconds);

    super.RPS = flywheelMotorSim.getAngularVelocityRPM() / 60.0;
    super.statorCurrent = flywheelMotorSim.getCurrentDrawAmps();
    super.isFlywheelAtSetpoint =
        Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance;

    DogLog.log("Flywheels/RPS", super.RPS);
    DogLog.log("Flywheels/TargetRPS", super.targetRPS);
    DogLog.log("Flywheels/statorCurrent", super.statorCurrent);
    DogLog.log("Flywheels/IsFlywheelAtSpeed", super.isFlywheelAtSetpoint);
  }

  @Override
  public void setFlywheelsRPS(double RPS) {
    super.targetRPS = RPS;
    double voltage =
        velocityController.calculate(super.RPS, RPS) + feedforward.calculate(RPS);
    setFlywheelsVoltage(voltage);
  }

  @Override
  public void setFlywheelsVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    flywheelMotorSim.setInputVoltage(clampedVoltage);
  }

  @Override
  public void stopFlywheels() {
    super.targetRPS = 0.0;
    velocityController.reset();
    setFlywheelsVoltage(0.0);
  }
}
