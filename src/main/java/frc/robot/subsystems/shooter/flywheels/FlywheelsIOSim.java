package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelsIOSim extends FlywheelsIO{
    private final DCMotorSim flywheelMotorSim;
    // private final SimpleMotorFeedforward feedforward;
    private final ProfiledPIDController pid;

    public FlywheelsIOSim() {
     flywheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60Foc(1), FlywheelsConstants.flywheelMOI, 
        FlywheelsConstants.flywheelGearRatio), DCMotor.getKrakenX60Foc(1));

      // feedforward = new SimpleMotorFeedforward(FlywheelsConstants.flywheelSimkS, 
      // FlywheelsConstants.flywheelSimkV);
      
      pid = new ProfiledPIDController(FlywheelsConstants.flywheelSimkP, 
      FlywheelsConstants.flywheelSimkI, FlywheelsConstants.flywheelSimkD, 
      new Constraints(FlywheelsConstants.flywheelMaxVelocity, FlywheelsConstants.flywheelMaxAcceleration));
    }

    @Override
    public void updateInputs(){
    flywheelMotorSim.update(0.02);

    super.statorCurrent = flywheelMotorSim.getCurrentDrawAmps();
    super.position = flywheelMotorSim.getAngularPositionRad();
    super.RPM = flywheelMotorSim.getAngularVelocityRPM();
 

    DogLog.log("Flywheel/Velocity", super.RPM);
    DogLog.log("Flywheel/Position", super.position);
    DogLog.log("Flywheel/Stator Current", super.statorCurrent);
  }

  @Override
  public void setVoltage(double voltage) {
    flywheelMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
 }

 @Override
 public void setRPM(double RPM) {
setVoltage(
        // feedforward.calculate(RPM)
        //     +
        pid.calculate(flywheelMotorSim.getAngularVelocityRPM(), super.targetRPM
        ));
 }
}
