package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Motor.TalonFXWrapper;

public class FlywheelsIOTalonFX extends FlywheelsIO {

    private final TalonFXWrapper flywheelLeaderWrapper;
    private final TalonFXWrapper flywheelBottomLeftWrapper;
    private final TalonFXWrapper flywheelTopRightWrapper;
    private final TalonFXWrapper flywheelBottomRightWrapper;

    private final TalonFX flywheelLeaderMotor;
    private final TalonFX flywheelBottomLeftMotor;
    private final TalonFX flywheelTopRightMotor;
    private final TalonFX flywheelBottomRightMotor;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

    //status signals
    private final StatusSignal<AngularVelocity> flywheelLeaderVelocityRPS;
    private final StatusSignal<AngularVelocity> flywheelFollower1VelocityRPS;
    private final StatusSignal<AngularVelocity> flywheelFollower2VelocityRPS;
    private final StatusSignal<AngularVelocity> flywheelFollower3VelocityRPS;
    private final StatusSignal<Current> flywheelLeaderStatorCurrent;
    private final StatusSignal<Current> flywheelFollower1StatorCurrent;
    private final StatusSignal<Current> flywheelFollower2StatorCurrent;
    private final StatusSignal<Current> flywheelFollower3StatorCurrent;
    private final StatusSignal<Current> flywheelLeaderSupplyCurrent;
    private final StatusSignal<Current> flywheelFollower1SupplyCurrent;
    private final StatusSignal<Current> flywheelFollower2SupplyCurrent;
    private final StatusSignal<Current> flywheelFollower3SupplyCurrent;
    private final StatusSignal<Voltage> flywheelLeaderAppliedVolts;
    private final StatusSignal<Voltage> flywheelFollower1AppliedVolts;
    private final StatusSignal<Voltage> flywheelFollower2AppliedVolts;
    private final StatusSignal<Voltage> flywheelFollower3AppliedVolts;
    private final BaseStatusSignal[] characterizationSignals;

    private final Debouncer rpmDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    public FlywheelsIOTalonFX() {
        //motor stuff
        flywheelLeaderWrapper = new TalonFXWrapper(FlywheelsConstants.flywheelLeader);
        flywheelBottomLeftWrapper = new TalonFXWrapper(FlywheelsConstants.flywheelBottomLeft);
        flywheelTopRightWrapper = new TalonFXWrapper(FlywheelsConstants.flywheelTopRight);
        flywheelBottomRightWrapper = new TalonFXWrapper(FlywheelsConstants.flywheelBottomRight);

        flywheelLeaderMotor = flywheelLeaderWrapper.getTalonFX();
        flywheelBottomLeftMotor = flywheelBottomLeftWrapper.getTalonFX();
        flywheelTopRightMotor = flywheelTopRightWrapper.getTalonFX();
        flywheelBottomRightMotor = flywheelBottomRightWrapper.getTalonFX();

        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(super.targetRPS);

        flywheelBottomLeftMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        flywheelTopRightMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        flywheelBottomRightMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        //status signal stuff
        flywheelLeaderVelocityRPS = flywheelLeaderMotor.getVelocity();
        flywheelFollower1VelocityRPS = flywheelBottomLeftMotor.getVelocity();
        flywheelFollower2VelocityRPS = flywheelTopRightMotor.getVelocity();
        flywheelFollower3VelocityRPS = flywheelBottomRightMotor.getVelocity();
        
        flywheelLeaderStatorCurrent = flywheelLeaderMotor.getStatorCurrent();
        flywheelFollower1StatorCurrent = flywheelBottomLeftMotor.getStatorCurrent();
        flywheelFollower2StatorCurrent = flywheelTopRightMotor.getStatorCurrent();
        flywheelFollower3StatorCurrent = flywheelBottomRightMotor.getStatorCurrent();
        
        flywheelLeaderSupplyCurrent = flywheelLeaderMotor.getSupplyCurrent();
        flywheelFollower1SupplyCurrent = flywheelBottomLeftMotor.getSupplyCurrent();
        flywheelFollower2SupplyCurrent = flywheelTopRightMotor.getSupplyCurrent();
        flywheelFollower3SupplyCurrent = flywheelBottomRightMotor.getSupplyCurrent();
        
        flywheelLeaderAppliedVolts = flywheelLeaderMotor.getMotorVoltage();
        flywheelFollower1AppliedVolts = flywheelBottomLeftMotor.getMotorVoltage();
        flywheelFollower2AppliedVolts = flywheelTopRightMotor.getMotorVoltage();
        flywheelFollower3AppliedVolts = flywheelBottomRightMotor.getMotorVoltage();

        characterizationSignals = new BaseStatusSignal[] {
            flywheelLeaderVelocityRPS, flywheelFollower1VelocityRPS, flywheelFollower2VelocityRPS, flywheelFollower3VelocityRPS,
            flywheelLeaderStatorCurrent, flywheelFollower1StatorCurrent, flywheelFollower2StatorCurrent, flywheelFollower3StatorCurrent,
            flywheelLeaderSupplyCurrent, flywheelFollower1SupplyCurrent, flywheelFollower2SupplyCurrent, flywheelFollower3SupplyCurrent,
            flywheelLeaderAppliedVolts, flywheelFollower1AppliedVolts, flywheelFollower2AppliedVolts, flywheelFollower3AppliedVolts
        };

        BaseStatusSignal.setUpdateFrequencyForAll(50, characterizationSignals);

        flywheelLeaderMotor.optimizeBusUtilization();
        flywheelBottomLeftMotor.optimizeBusUtilization();
        flywheelTopRightMotor.optimizeBusUtilization();
        flywheelBottomRightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(characterizationSignals);

        super.RPS = flywheelLeaderVelocityRPS.getValueAsDouble();
        super.statorCurrent = flywheelLeaderStatorCurrent.getValueAsDouble();
        super.supplyCurrent = flywheelLeaderSupplyCurrent.getValueAsDouble();
        super.isFlywheelAtSetpoint = rpmDebouncer.calculate(
            Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);

        DogLog.log("Flywheels/RPS", super.RPS);
        DogLog.log("Flywheels/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
        DogLog.log("Flywheels/isFlywheelAtSpeedRaw",
            Math.abs(super.RPS - super.targetRPS) < FlywheelsConstants.RPSTolerance);
        DogLog.log("Flywheels/statorCurrent", super.statorCurrent);
        DogLog.log("Flywheels/supplyCurrent", super.supplyCurrent);
        DogLog.log("Flywheels/temperature", super.temperature);
    }

    @Override
    public void setFlywheelsRPS(double RPS) {
        super.targetRPS = RPS;
        flywheelLeaderMotor.setControl(velocityTorqueCurrentFOC.withVelocity(RPS));
    }

    @Override
    public void setFlywheelsVoltage(double voltage) {
        flywheelLeaderWrapper.setVoltage(voltage);
    }

    @Override
    public void stopFlywheels() {
        flywheelLeaderWrapper.stop();
    }
}
