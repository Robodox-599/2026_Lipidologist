package frc.robot.subsystems.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;



public class LEDsIOReal extends LEDsIO{
    public CANBus canBus;
    public CANdle candle;
    public CANdleConfiguration candleConfig;

    public LEDsIOReal(){
        this.canBus = new CANBus(LEDsConstants.ledsCanBus);
        this.candle = new CANdle(LEDsConstants.candleID, this.canBus);
        this.candleConfig = new CANdleConfiguration();

        this.candleConfig.LED.StripType = StripTypeValue.RGB;
        
        this.candle.getConfigurator().apply(this.candleConfig);
    }

    @Override
    public void setLedColor(int red, int green, int blue){
        RGBWColor color = new RGBWColor(red, green, blue, 0);

        this.candle.setControl(new SolidColor(0, LEDsConstants.ledEnd).withColor(color));
    }
}
