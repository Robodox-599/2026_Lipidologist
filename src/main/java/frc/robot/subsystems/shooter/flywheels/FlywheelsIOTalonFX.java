package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class FlywheelsIOTalonFX extends FlywheelsIO {
    // motors right
    private final CANBus flywheelRightCANBus;
    private final TalonFX flywheelRightMotor;
    TalonFXConfiguration flywheelRightConfiguration;

    //motor center
    private final CANBus flywheelCenterCANBus;
    private final TalonFX flywheelCenterMotor;
    TalonFXConfiguration flywheelCenterConfiguration;

    //motor left
    private final CANBus flywheelLeftCANBus;
    private final TalonFX flywheelLeftMotor;
    TalonFXConfiguration flywheelLeftConfiguration;

    // status signals
    //right
    private final StatusSignal<AngularVelocity> flywheelVelocityRPSRight;
    private final StatusSignal<Temperature> flywheelTemperatureRight;
    private final StatusSignal<Angle> flywheelPositionRight;
    private final StatusSignal<Voltage> flywheelAppliedVoltsRight;
    private final StatusSignal<Current> flywheelStatorCurrentRight;
    private final StatusSignal<Current> flywheelSupplyCurrentRight;

    //center
    private final StatusSignal<AngularVelocity> flywheelVelocityRPSCenter;
    private final StatusSignal<Temperature> flywheelTemperatureCenter;
    private final StatusSignal<Angle> flywheelPositionCenter;
    private final StatusSignal<Voltage> flywheelAppliedVoltsCenter;
    private final StatusSignal<Current> flywheelStatorCurrentCenter;
    private final StatusSignal<Current> flywheelSupplyCurrentCenter;

    //left
    private final StatusSignal<AngularVelocity> flywheelVelocityRPSLeft;
    private final StatusSignal<Temperature> flywheelTemperatureLeft;
    private final StatusSignal<Angle> flywheelPositionLeft;
    private final StatusSignal<Voltage> flywheelAppliedVoltsLeft;
    private final StatusSignal<Current> flywheelStatorCurrentLeft;
    private final StatusSignal<Current> flywheelSupplyCurrentLeft;

    public FlywheelsIOTalonFX() {
        //motors + configuration
        flywheelRightCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);
        flywheelCenterCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);
        flywheelLeftCANBus = new CANBus(FlywheelsConstants.flywheelCANBus);

        flywheelRightMotor = new TalonFX(FlywheelsConstants.flywheelMotorIDRight, flywheelRightCANBus);
        flywheelCenterMotor = new TalonFX(FlywheelsConstants.flywheelMotorIDRight, flywheelCenterCANBus);        
        flywheelLeftMotor = new TalonFX(FlywheelsConstants.flywheelMotorIDRight, flywheelLeftCANBus);

        flywheelRightConfiguration = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withPeakReverseDutyCycle(0))
                .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(0))
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(FlywheelsConstants.supplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
                                .withStatorCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(FlywheelsConstants.flywheelRightkP)
                                .withKI(FlywheelsConstants.flywheelRightkI)
                                .withKD(FlywheelsConstants.flywheelRightkD)
                                .withKS(FlywheelsConstants.flywheelRightkS)
                                .withKV(FlywheelsConstants.flywheelRightkV))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        flywheelCenterConfiguration = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withPeakReverseDutyCycle(0))
                .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(0))
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(FlywheelsConstants.supplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
                                .withStatorCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(FlywheelsConstants.flywheelCenterkP)
                                .withKI(FlywheelsConstants.flywheelCenterkI)
                                .withKD(FlywheelsConstants.flywheelCenterkD)
                                .withKS(FlywheelsConstants.flywheelCenterkS)
                                .withKV(FlywheelsConstants.flywheelCenterkV))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

         flywheelLeftConfiguration = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withPeakReverseDutyCycle(0))
                .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(0))
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(FlywheelsConstants.supplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true)
                                .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
                                .withStatorCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(FlywheelsConstants.flywheelLeftkP)
                                .withKI(FlywheelsConstants.flywheelLeftkI)
                                .withKD(FlywheelsConstants.flywheelLeftkD)
                                .withKS(FlywheelsConstants.flywheelLeftkS)
                                .withKV(FlywheelsConstants.flywheelLeftkV))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        // Applying configuration
        PhoenixUtil.tryUntilOk(10, () -> flywheelRightMotor.getConfigurator().apply(flywheelRightConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> flywheelCenterMotor.getConfigurator().apply(flywheelCenterConfiguration, 1));
        PhoenixUtil.tryUntilOk(10, () -> flywheelLeftMotor.getConfigurator().apply(flywheelLeftConfiguration, 1));



        // status signals
        flywheelVelocityRPSRight = flywheelRightMotor.getVelocity();
        flywheelTemperatureRight = flywheelRightMotor.getDeviceTemp();
        flywheelPositionRight = flywheelRightMotor.getPosition();
        flywheelAppliedVoltsRight = flywheelRightMotor.getMotorVoltage();
        flywheelStatorCurrentRight = flywheelRightMotor.getStatorCurrent();
        flywheelSupplyCurrentRight = flywheelRightMotor.getSupplyCurrent();

        flywheelVelocityRPSCenter = flywheelCenterMotor.getVelocity();
        flywheelTemperatureCenter = flywheelCenterMotor.getDeviceTemp();
        flywheelPositionCenter = flywheelCenterMotor.getPosition();
        flywheelAppliedVoltsCenter = flywheelCenterMotor.getMotorVoltage();
        flywheelStatorCurrentCenter = flywheelCenterMotor.getStatorCurrent();
        flywheelSupplyCurrentCenter = flywheelCenterMotor.getSupplyCurrent();

        flywheelVelocityRPSLeft = flywheelLeftMotor.getVelocity();
        flywheelTemperatureLeft = flywheelLeftMotor.getDeviceTemp();
        flywheelPositionLeft = flywheelLeftMotor.getPosition();
        flywheelAppliedVoltsLeft = flywheelLeftMotor.getMotorVoltage();
        flywheelStatorCurrentLeft = flywheelLeftMotor.getStatorCurrent();
        flywheelSupplyCurrentLeft = flywheelLeftMotor.getSupplyCurrent();

        //update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(50, 
            flywheelVelocityRPSRight, flywheelTemperatureRight, flywheelPositionRight, 
            flywheelAppliedVoltsRight, flywheelStatorCurrentRight, flywheelSupplyCurrentRight,
                
            flywheelVelocityRPSCenter, flywheelTemperatureCenter, flywheelPositionCenter, 
            flywheelAppliedVoltsCenter, flywheelStatorCurrentCenter,flywheelSupplyCurrentCenter,
                
            flywheelVelocityRPSLeft, flywheelTemperatureLeft, flywheelPositionLeft, 
            flywheelAppliedVoltsLeft, flywheelStatorCurrentLeft, flywheelSupplyCurrentLeft
        );

        flywheelRightMotor.optimizeBusUtilization();
        flywheelCenterMotor.optimizeBusUtilization();
        flywheelLeftMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs() {
        // BaseStatusSignal.refreshAll(flywheelVelocityRPS, flywheelTemperature,
        //         flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

        super.RPSRight = flywheelVelocityRPSRight.getValueAsDouble();
        super.statorCurrentRight = flywheelStatorCurrentRight.getValueAsDouble();
        super.supplyCurrentRight = flywheelSupplyCurrentRight.getValueAsDouble();
        super.temperatureRight = flywheelTemperatureRight.getValueAsDouble();
        super.isFlywheelAtSetpointRight = Math.abs(super.RPSRight - super.targetRPS) < FlywheelsConstants.RPSTolerance;
        
        super.RPSCenter = flywheelVelocityRPSCenter.getValueAsDouble();
        super.statorCurrentCenter = flywheelStatorCurrentCenter.getValueAsDouble();
        super.supplyCurrentCenter = flywheelSupplyCurrentCenter.getValueAsDouble();
        super.temperatureCenter = flywheelTemperatureCenter.getValueAsDouble();
        super.isFlywheelAtSetpointCenter = Math.abs(super.RPSCenter - super.targetRPS) < FlywheelsConstants.RPSTolerance;
        
        super.RPSLeft = flywheelVelocityRPSLeft.getValueAsDouble();
        super.statorCurrentLeft = flywheelStatorCurrentLeft.getValueAsDouble();
        super.supplyCurrentLeft = flywheelSupplyCurrentLeft.getValueAsDouble();
        super.temperatureLeft = flywheelTemperatureLeft.getValueAsDouble();
        super.isFlywheelAtSetpointLeft = Math.abs(super.RPSLeft - super.targetRPS) < FlywheelsConstants.RPSTolerance;

        DogLog.log("Flywheels/TargetRPS", super.targetRPS);

        DogLog.log("Flywheels/" + "Right" + "/RPS", super.RPSRight);
        DogLog.log("Flywheels/" + "Right" + "/isFlywheelAtSpeed", super.isFlywheelAtSetpointRight);
        DogLog.log("Flywheels/" + "Right" + "/statorCurrent", super.statorCurrentRight);
        DogLog.log("Flywheels/" + "Right" + "/supplyCurrent", super.supplyCurrentRight);
        DogLog.log("Flywheels/" + "Right" + "/temperature", super.temperatureRight);
        
        DogLog.log("Flywheels/" + "Center" + "/RPS", super.RPSCenter);
        DogLog.log("Flywheels/" + "Center" + "/isFlywheelAtSpeed", super.isFlywheelAtSetpointCenter);
        DogLog.log("Flywheels/" + "Center" + "/statorCurrent", super.statorCurrentCenter);
        DogLog.log("Flywheels/" + "Center" + "/supplyCurrent", super.supplyCurrentCenter);
        DogLog.log("Flywheels/" + "Center" + "/temperature", super.temperatureCenter);

        DogLog.log("Flywheels/" + "Left" + "/RPS", super.RPSLeft);
        DogLog.log("Flywheels/" + "Left" + "/isFlywheelAtSpeed", super.isFlywheelAtSetpointLeft);
        DogLog.log("Flywheels/" + "Left" + "/statorCurrent", super.statorCurrentLeft);
        DogLog.log("Flywheels/" + "Left" + "/supplyCurrent", super.supplyCurrentLeft);
        DogLog.log("Flywheels/" + "Left" + "/temperature", super.temperatureLeft);
    }

    @Override
    public void setRPS(double RPS) {
        super.targetRPS = RPS;
        flywheelRightMotor.setControl(new VelocityVoltage(super.targetRPS));
        flywheelCenterMotor.setControl(new VelocityVoltage(super.targetRPS));
        flywheelLeftMotor.setControl(new VelocityVoltage(super.targetRPS));

    }

    @Override
    public void setVoltage(double voltage) {
        flywheelRightMotor.setVoltage(voltage);
        flywheelCenterMotor.setVoltage(voltage);
        flywheelLeftMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        flywheelRightMotor.setVoltage(0);
        flywheelCenterMotor.setVoltage(0);
        flywheelLeftMotor.setVoltage(0);
    }
}


    // public FlywheelsIOTalonFX(FlywheelConstants flywheelConstants) {
    //     this.flywheelConstants = flywheelConstants;

    //     // motors + configuration
    //     flywheelCANBus = new CANBus(this.flywheelConstants.CANBus());
    //     flywheelMotor = new TalonFX(this.flywheelConstants.motorID(), flywheelCANBus);
        // flywheelConfiguration = new TalonFXConfiguration()
        //         .withMotorOutput(
        //                 new MotorOutputConfigs()
        //                         .withPeakReverseDutyCycle(0))
        //         .withTorqueCurrent(new TorqueCurrentConfigs().withPeakReverseTorqueCurrent(0))
        //         .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
        //         .withCurrentLimits(
        //                 new CurrentLimitsConfigs()
        //                         .withSupplyCurrentLimit(FlywheelsConstants.supplyCurrentLimit)
        //                         .withSupplyCurrentLimitEnable(true)
        //                         .withStatorCurrentLimit(FlywheelsConstants.statorCurrentLimit)
        //                         .withStatorCurrentLimitEnable(true))
        //         .withSlot0(
        //                 new Slot0Configs()
        //                         .withKP(this.flywheelConstants.kP())
        //                         .withKI(this.flywheelConstants.kI())
        //                         .withKD(this.flywheelConstants.kD())
        //                         .withKS(this.flywheelConstants.kS())
        //                         .withKV(this.flywheelConstants.kV()))
        //         .withMotorOutput(new MotorOutputConfigs().withInverted(this.flywheelConstants.invert())
        //                 .withNeutralMode(NeutralModeValue.Coast));

    //     // Applying configuration
    //     PhoenixUtil.tryUntilOk(10, () -> flywheelMotor.getConfigurator().apply(flywheelConfiguration, 1));

    //     // status signals
    //     flywheelVelocityRPS = flywheelMotor.getVelocity();
    //     flywheelTemperature = flywheelMotor.getDeviceTemp();
    //     flywheelPosition = flywheelMotor.getPosition();
    //     flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
    //     flywheelStatorCurrent = flywheelMotor.getStatorCurrent();
    //     flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();

    //     // update frequency
    //     // BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocityRPS, flywheelTemperature,
    //     //         flywheelPosition, flywheelAppliedVolts, flywheelStatorCurrent, flywheelSupplyCurrent);

    //     // flywheelMotor.optimizeBusUtilization();
    // }