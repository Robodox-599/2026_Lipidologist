package frc.robot.subsystems.shooter.flywheels;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelsIOSim extends FlywheelsIO{
    private final DCMotorSim flywheelMotorSim;
    private final ProfiledPIDController pid;

    public FlywheelsIOSim() {
     flywheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60Foc(1), FlywheelsConstants.flywheelMOI, 
        FlywheelsConstants.flywheelGearRatio), DCMotor.getKrakenX60Foc(1));
      // FlywheelsConstants.flywheelSimkV);
      
      pid = new ProfiledPIDController(FlywheelsConstants.flywheelSimkP, 
      FlywheelsConstants.flywheelSimkI, FlywheelsConstants.flywheelSimkD, 
      new Constraints(FlywheelsConstants.flywheelMaxVelocity, FlywheelsConstants.flywheelMaxAcceleration));
    }

    @Override
    public void updateInputs(){
    flywheelMotorSim.update(0.02);

    super.RPM = flywheelMotorSim.getAngularVelocityRPM();
    super.isFlywheelAtSetpoint = 
        Math.abs(super.RPM - super.targetRPM) < FlywheelsConstants.RPMTolerance;
 
    DogLog.log("Flywheel/RPM", super.RPM);
    DogLog.log("Flywheel/TargetRPM", super.targetRPM);
    DogLog.log("Flywheel/IsFlywheelAtSpeed", super.isFlywheelAtSetpoint);
  }

  @Override
  public void setVoltage(double voltage) {
    flywheelMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
 }

 @Override
 public void setRPM(double RPM) {
setVoltage(pid.calculate(flywheelMotorSim.getAngularVelocityRPM(), RPM));
 }

  @Override
 public void stop() {
  super.targetRPM =0;
    flywheelMotorSim.setInputVoltage(pid.calculate(flywheelMotorSim.getAngularVelocityRPM(), 0));
 }
}
