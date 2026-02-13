package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIO{
    private final DCMotorSim hoodMotorSim;
    private final ProfiledPIDController pid;

    public HoodIOSim() {
     hoodMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60Foc(1), HoodConstants.hoodMOI, 
        HoodConstants.hoodGearRatio), DCMotor.getKrakenX60Foc(1));
      
      pid = new ProfiledPIDController(HoodConstants.hoodSimkP, 
      HoodConstants.hoodSimkI, HoodConstants.hoodSimkD, new Constraints(HoodConstants.hoodMaxVelocity, HoodConstants.hoodMaxAcceleration));
    }

    @Override
    public void updateInputs(){
    hoodMotorSim.update(0.02);

    super.positionRotations = hoodMotorSim.getAngularPositionRotations();
     super.isHoodInPosition = 
            Math.abs(super.positionRotations - super.targetPosition) < HoodConstants.positionTolerance;

    DogLog.log("Hood/Position", super.positionRotations);
    DogLog.log("Hood/TargetPosition", super.targetPosition);
    DogLog.log("Hood/IsHoodAtPosition", super.isHoodInPosition);

  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
 }

 @Override
 public void setPosition(double position) {
    hoodMotorSim.setInputVoltage(pid.calculate(hoodMotorSim.getAngularPositionRotations(), position));
 }

 @Override
 public void stop() {
    super.targetPosition = 0;
    hoodMotorSim.setInputVoltage(pid.calculate(hoodMotorSim.getAngularPositionRotations(), 0));
 }
}

