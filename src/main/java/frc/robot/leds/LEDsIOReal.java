package frc.robot.leds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.RGBWColor;

import dev.doglog.DogLog;


public class LEDsIOReal extends LEDsIO{
    public final CANdle candleReal;
    public final CANBus CANbus;
    public final CANdleConfiguration candleConfig;

public LEDsIOReal() {
    CANbus = new CANBus(LEDsConstants.CANbus);
    candleReal = new CANdle(LEDsConstants.CANdleID, CANbus);
    candleConfig = new CANdleConfiguration()
        .withCANdleFeatures(
            new CANdleFeaturesConfigs()
                .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled))
        .withLED(
            new LEDConfigs()
                .withBrightnessScalar(LEDsConstants.colorScaler)
                .withStripType(StripTypeValue.GRB)
                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs));
    
    candleReal.getConfigurator().apply(candleConfig);
    }

    @Override
    public void updateInputs(){
        super.areLEDsConnected = true;
        DogLog.log("LEDs/Connected", super.areLEDsConnected);
    }

    @Override
    public void setLEDsColor(RGBWColor color) {
        candleReal.setControl(new StrobeAnimation(0, LEDsConstants.maxLEDs)
            .withColor(color).withFrameRate(LEDsConstants.frameRate));
    }
}
