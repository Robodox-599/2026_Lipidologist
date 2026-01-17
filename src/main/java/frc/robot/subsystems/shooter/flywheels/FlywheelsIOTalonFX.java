package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelsIOTalonFX extends FlywheelsIO{
    //motors + configuration
    private final TalonFX flywheelMotor;
    TalonFXConfiguration flywheelConfiguration;
   
    //status signals
    private final StatusSignal<AngularVelocity> flywheelVelocityRad;
    private final StatusSignal<Temperature> flywheelTemperature;
    private final StatusSignal<Angle> flywheelPosition;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelStatorCurrent;
    private final StatusSignal<Current> flywheelSupplyCurrent;

    public FlywheelsIOTalonFX(){
        //motors + configuration
        flywheelMotor = new TalonFX(FlywheelsConstants.flywheelMotorID, FlywheelsConstants.flywheelCANBus);
        flywheelConfiguration = new TalonFXConfiguration();  
        
        //applying PID to configuration
        flywheelConfiguration.Slot0.kP = FlywheelsConstants.flywheelRealkP;
        flywheelConfiguration.Slot0.kI = FlywheelsConstants.flywheelRealkI;
        flywheelConfiguration.Slot0.kD = FlywheelsConstants.flywheelRealkD;
        flywheelConfiguration.Slot0.kS = FlywheelsConstants.flywheelRealkS;
        flywheelConfiguration.Slot0.kV = FlywheelsConstants.flywheelRealkV;
        
        //current limits
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimit = FlywheelsConstants.supplyCurrentLimit;

        //other configuration stuff
        flywheelMotor.setNeutralMode(NeutralModeValue.Brake);

        //Applying configuration
        flywheelMotor.getConfigurator().apply(flywheelConfiguration);
        flywheelMotor.setNeutralMode(NeutralModeValue.Brake);

        //status signals
        flywheelVelocityRad = flywheelMotor.getVelocity();
        flywheelTemperature = flywheelMotor.getDeviceTemp();
        flywheelPosition = flywheelMotor.getPosition();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelStatorCurrent = flywheelMotor.getStatorCurrent();
        flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocityRad, flywheelTemperature, 
        flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        flywheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(){
        BaseStatusSignal.refreshAll(flywheelVelocityRad, flywheelTemperature, 
        flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        super.position = flywheelPosition.getValueAsDouble();
        super.velocity = flywheelVelocityRad.getValueAsDouble();
        super.isFlywheelAtSetpoint = 
            Math.abs(super.velocity - super.velocitySetpoint) < FlywheelsConstants.velocityTolerance;

        DogLog.log("Flywheel/Position", super.position);
        DogLog.log("FLywheel/Velocity", super.velocity);
        DogLog.log("Flywheel/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
    }

    @Override
    public void setRPM(double RPM){
        super.velocitySetpoint = velocity;
        flywheelMotor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }
}
