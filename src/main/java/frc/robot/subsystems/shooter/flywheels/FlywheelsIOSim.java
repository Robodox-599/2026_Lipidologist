package frc.robot.subsystems.shooter.flywheels;

// import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.FlywheelConstants;

public class FlywheelsIOSim extends FlywheelsIO {
  //   private final DCMotorSim flywheelMotorSim;
  //   private final ProfiledPIDController pid;

  //   private final FlywheelConstants flywheelConstants;

  public FlywheelsIOSim() {
    //     this.flywheelConstants = flywheelConstants;

    //     flywheelMotorSim = new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),
    // FlywheelsConstants.flywheelMOI,
    //             FlywheelsConstants.flywheelGearRatio),
    //         DCMotor.getKrakenX60Foc(1));

    //     pid = new ProfiledPIDController(this.flywheelConstants.kP(),
    //         this.flywheelConstants.kI(), this.flywheelConstants.kD(),
    //         new Constraints(FlywheelsConstants.flywheelMaxVelocity,
    // FlywheelsConstants.flywheelMaxAcceleration));

  }

  //   @Override
  //   public void updateInputs() {
  //     flywheelMotorSim.update(0.02);

  //     super.RPS = flywheelMotorSim.getAngularVelocityRPM() / 60.0;
  //     super.statorCurrent = flywheelMotorSim.getCurrentDrawAmps();
  //     super.isFlywheelAtSetpoint = Math.abs(super.RPS - super.targetRPS) <
  // FlywheelsConstants.RPSTolerance;

  //     DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/RPS", super.RPS);
  //     DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/TargetRPS", super.targetRPS);
  //     DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/StatorCurrent",
  // super.statorCurrent);
  //     DogLog.log("Flywheels/" + this.flywheelConstants.name() + "/IsFlywheelAtSpeed",
  // super.isFlywheelAtSetpoint);
  //   }

  //   @Override
  //   public void setVoltage(double voltage) {
  //     flywheelMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  //   }

  //   @Override
  //   public void setRPS(double RPS) {
  //     super.targetRPS = RPS;
  //     setVoltage(pid.calculate(flywheelMotorSim.getAngularVelocityRPM() / 60.0,
  // super.targetRPS));
  //   }

  //   @Override
  //   public void stop() {
  //     super.targetRPS = 0;
  //     setVoltage(0);
  //   }
}
