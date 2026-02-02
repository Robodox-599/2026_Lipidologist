package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbIOSim extends ClimbIO {
  private final PIDController positionController;
  //private final ElevatorFeedforward feedForward;
  private double targetPositionInches = 0.0;


  private static final ElevatorSim climbSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          6,
          Units.lbsToKilograms(28),
          Units.inchesToMeters(1),
          Units.inchesToMeters(0),
          Units.inchesToMeters(89),
          true,
          0);

  public ClimbIOSim() {
    positionController =
        new PIDController(
            ClimbConstants.simKP, ClimbConstants.simKI, ClimbConstants.simKD);
    //feedForward = new ElevatorFeedforward(ClimbConstants.simKS, ClimbConstants.simKG, ClimbConstants.simKS);
    positionController.setTolerance(ClimbConstants.positionToleranceInches);
  }

  @Override
  public void updateInputs() {
    climbSim.update(0.02);

    super.velocityInchesPerSec = (Units.metersToInches(climbSim.getVelocityMetersPerSecond()));
    super.targetPositionInches = targetPositionInches;
    super.positionInches = (Units.metersToInches(climbSim.getPositionMeters()));
    super.appliedVolts = climbSim.getCurrentDrawAmps();
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
    climbSim.setInputVoltage(voltage);
  }

  @Override
  public void zeroClimbPosition() {
    targetPositionInches = 0.0;
  }
}