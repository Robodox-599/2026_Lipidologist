package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim extends ClimbIO {
  private final ProfiledPIDController positionController;
  private double targetPositionInches = 0.0;

  private final DCMotorSim climbSim;
  private static final DCMotor climbGearbox = DCMotor.getKrakenX60Foc(1);

  public ClimbIOSim() {
    positionController =
        new ProfiledPIDController(
            ClimbConstants.simKP, ClimbConstants.simKI, ClimbConstants.simKD, 
              new TrapezoidProfile.Constraints(ClimbConstants.maxVelocityRotsPerSec, ClimbConstants.maxAccelerationRotationsPerSecSQ));

    climbSim = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (climbGearbox, ClimbConstants.climbMOI, ClimbConstants.climbGearRatio), climbGearbox);

    positionController.setTolerance(ClimbConstants.positionToleranceInches);
  }

  @Override
  public void updateInputs() {
    climbSim.update(0.02);

    super.velocityInchesPerSec = (climbSim.getAngularVelocityRPM() * ClimbConstants.inchesPerRev) / 60.0;
    super.targetPositionInches = targetPositionInches;
    super.positionInches = (climbSim.getAngularPositionRad() / (2 * Math.PI)) * ClimbConstants.inchesPerRev;
    super.tempCelsius = 25.0;

    DogLog.log("Climb/VelocityInchesPerSec", super.velocityInchesPerSec);
    DogLog.log("Climb/TargetPositionInches", super.targetPositionInches);
    DogLog.log("Climb/PositionInches", super.positionInches);
    DogLog.log("Climb/AppliedVoltages", super.appliedVolts);
    DogLog.log("Climb/TempCelcius", super.tempCelsius);
  }

  @Override
  public void setClimbHeight(double height) {
    targetPositionInches = 
      MathUtil.clamp
        (ClimbConstants.convertToTicks(height), ClimbConstants.climbLowerLimit, ClimbConstants.climbUpperLimit);

    climbSim.setInputVoltage(positionController.calculate(super.positionInches, super.targetPositionInches));  
  }

  @Override
  public void stopClimb() {
    climbSim.setInputVoltage(0);
  }

  @Override
  public void setClimbVoltage(double voltage) {
    super.appliedVolts = voltage;
    climbSim.setInputVoltage(voltage);

  }

  @Override
  public void zeroClimbPosition() {
    targetPositionInches = 0.0;
  }
}