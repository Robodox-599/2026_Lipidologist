package frc.robot.subsystems.shooter.hood;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIO{
    private final DCMotorSim hoodMotorSim;
    private final SimpleMotorFeedforward feedforward;
    private final ProfiledPIDController pid;

    public HoodIOSim() {
     hoodMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem
      (DCMotor.getKrakenX60Foc(1), HoodConstants.hoodMOI, 
        HoodConstants.hoodGearRatio), DCMotor.getKrakenX60Foc(1));

      feedforward = new SimpleMotorFeedforward(HoodConstants.hoodSimkS, 
      HoodConstants.hoodSimkV);
      
      pid = new ProfiledPIDController(HoodConstants.hoodSimkP, 
      HoodConstants.hoodSimkI, HoodConstants.hoodSimkD, null);
    }

    @Override
    public void updateInputs(){
    hoodMotorSim.update(0.02);

    super.positionRadians = hoodMotorSim.getAngularPositionRad();
    super.velocity = hoodMotorSim.getAngularVelocityRPM() / 60.0;
 

    DogLog.log("Hood/Velocity", super.velocity);
    DogLog.log("Hood/Position", super.positionRadians);
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
 }

 @Override
 public void setPosition(double position) {
    hoodMotorSim.setAngle(feedforward.calculate(position)
            + pid.calculate(hoodMotorSim.getAngularVelocityRPM(), position));
 }
}

