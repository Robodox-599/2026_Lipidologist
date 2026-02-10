package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FlywheelsIOTalonFX extends FlywheelsIO{
    //motors + configuration
    private final CANBus flywheelCANBus;
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
        flywheelCANBus = new CANBus();
        flywheelMotor = new TalonFX(FlywheelsConstants.flywheelMotorID, flywheelCANBus);
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
        PhoenixUtil.tryUntilOk(10, () -> flywheelMotor.getConfigurator().apply(flywheelConfiguration, 1));

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
        super.RPM = flywheelVelocityRad.getValueAsDouble();
        super.isFlywheelAtSetpoint = 
            Math.abs(super.RPM - super.targetRPM) < FlywheelsConstants.RPMTolerance;

        DogLog.log("Flywheel/Position", super.position);
        DogLog.log("Flywheel/RPM", super.RPM);
        DogLog.log("Flywheel/isFlywheelAtSpeed", super.isFlywheelAtSetpoint);
    }

    @Override
    public void setRPM(double RPM){
        super.targetRPM = RPM;

        if(super.targetRPM < super.RPM){
            flywheelMotor.setControl(new StaticBrake());
        } else {
            flywheelMotor.setControl(new VelocityTorqueCurrentFOC(RPM));
        }
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
