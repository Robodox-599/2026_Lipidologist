package frc.robot.subsystems.intake.intakeWrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Motor.TalonFXWrapper;

public class IntakeWristIOTalonFX extends IntakeWristIO {
  private final TalonFXWrapper intakeWristMotorWrapper;
  public final TalonFX intakeWristMotor;
  public final CANBus intakeWristCanBus;
  private MotionMagicVoltage m_request;

  //status signal
  public final StatusSignal<Angle> intakeWristPosition;
  public final StatusSignal<AngularVelocity> intakeWristVelocity;
  public final StatusSignal<Voltage> intakeWristAppliedVolts;
  public final StatusSignal<Current> intakeWristStatorCurrent;
  public final StatusSignal<Current> intakeWristSupplyCurrent;
  public final StatusSignal<Temperature> intakeWristTemperature;
  private final Debouncer wristStallDebouncer =
      new Debouncer(IntakeWristConstants.debounceTime, DebounceType.kBoth);
    private final Debouncer wristHomeDebouncer =
      new Debouncer(IntakeWristConstants.debounceTime, DebounceType.kRising);

  public IntakeWristIOTalonFX() {
    intakeWristCanBus = new CANBus(IntakeWristConstants.intakeWristCanBus);
    
    intakeWristMotorWrapper = new TalonFXWrapper(IntakeWristConstants.intakeWristMotor);
    intakeWristMotor = intakeWristMotorWrapper.getTalonFX();

    m_request = new MotionMagicVoltage(0);

    intakeWristPosition = intakeWristMotor.getPosition();
    intakeWristVelocity = intakeWristMotor.getVelocity();
    intakeWristAppliedVolts = intakeWristMotor.getMotorVoltage();
    intakeWristStatorCurrent = intakeWristMotor.getStatorCurrent();
    intakeWristSupplyCurrent = intakeWristMotor.getSupplyCurrent();
    intakeWristTemperature = intakeWristMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeWristPosition,
        intakeWristVelocity,
        intakeWristStatorCurrent,
        intakeWristSupplyCurrent);
    
    BaseStatusSignal.setUpdateFrequencyForAll(
      5, 
      intakeWristTemperature,
      intakeWristAppliedVolts);

    intakeWristMotor.optimizeBusUtilization();
  }

  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        intakeWristPosition,
        intakeWristVelocity,
        intakeWristAppliedVolts,
        intakeWristStatorCurrent,
        intakeWristSupplyCurrent,
        intakeWristTemperature);

    super.currentPosition =
      intakeWristPosition.getValueAsDouble() / IntakeWristConstants.gearRatio;
    super.velocity = intakeWristVelocity.getValueAsDouble();
    super.voltage = intakeWristAppliedVolts.getValueAsDouble();
    super.statorCurrent = intakeWristStatorCurrent.getValueAsDouble();
    super.supplyCurrent = intakeWristSupplyCurrent.getValueAsDouble();
    super.temperature = intakeWristTemperature.getValueAsDouble();

    super.atSetpoint = Math.abs(super.currentPosition - super.targetPosition) < 0.02;
    super.isWristJammed =
        wristStallDebouncer.calculate(
            (super.statorCurrent > IntakeWristConstants.agitateStatorCurrentTrip)
                && (Math.abs(super.velocity) < IntakeWristConstants.velocityTrip));

    DogLog.log("Intake/Wrist/Position", super.currentPosition);
    DogLog.log("Intake/Wrist/TargetPosition", super.targetPosition);
    DogLog.log("Intake/Wrist/Velocity", super.velocity);
    DogLog.log("Intake/Wrist/AtSetpoint", super.atSetpoint);
    DogLog.log("Intake/Wrist/Voltage", super.voltage);
    DogLog.log("Intake/Wrist/StatorCurrent", super.statorCurrent);
    DogLog.log("Intake/Wrist/SupplyCurrent", super.supplyCurrent);
    DogLog.log("Intake/Wrist/Temperature", super.temperature);
  }

  @Override
  public void stopIntakeWrist() {
    intakeWristMotorWrapper.stop();
  }

  @Override
  public void setIntakeWristPosition(double position) {
    super.targetPosition = position;
    intakeWristMotor.setControl(
        m_request.withPosition(position * IntakeWristConstants.gearRatio).withEnableFOC(true));
  }

  @Override
  public void setIntakeWristVoltage(double voltage) {
    intakeWristMotorWrapper.setVoltage(voltage);
  }

  @Override
  public double getIntakeWristPosition() {
    return intakeWristPosition.getValueAsDouble() / IntakeWristConstants.gearRatio;
  }

  @Override
  public void homeIntakeWrist(){
    intakeWristMotor.setPosition(0);
  }

  @Override
  public boolean isWristReadyToHome(){
    return wristHomeDebouncer.calculate(
        super.statorCurrent > IntakeWristConstants.homeStatorCurrentTrip
            && Math.abs(super.velocity) < IntakeWristConstants.velocityTrip);
  }
}

