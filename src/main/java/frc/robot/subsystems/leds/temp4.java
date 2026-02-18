package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;



public class poo4 extends LEDsIO{
    public CANBus canBus;
    public CANdle candle;
    public CANdleConfiguration candleConfig;

    public poo4(){
        this.canBus = new CANBus(poo2.ledsCanBus);
        this.candle = new CANdle(poo2.candleID, this.canBus);
        this.candleConfig = new CANdleConfiguration();

        this.candleConfig.LED.StripType = StripTypeValue.RGB;
        
        this.candle.getConfigurator().apply(this.candleConfig);
    }

    @Override
    public void setLedColor(int red, int green, int blue){
        RGBWColor color = new RGBWColor(red, green, blue, 0);

        this.candle.setControl(new SolidColor(0, poo2.ledEnd).withColor(color));
    }
}
