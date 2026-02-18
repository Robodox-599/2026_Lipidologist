package frc.robot.leds;

import com.ctre.phoenix6.signals.RGBWColor;

public class LEDsConstants {
    //LED information
    public static final int CANdleID = 0;
    public static final String CANbus = "rio";
    public static final int maxLEDs = 0;

    public static final double colorScaler = 1;
    public static final double frameRate = 6;

    //state colors
    public static final RGBWColor transferingFuel =  new RGBWColor(0, 0, 0, 255); //white
    public static final RGBWColor hubShotPrepared =  new RGBWColor(255, 0, 255, 255); //purple
    public static final RGBWColor hubShooting =  new RGBWColor(255, 15, 122, 255); //pink
    public static final RGBWColor allianceZoneShotPrepared =  new RGBWColor(255, 255, 0, 255); //yellow
    public static final RGBWColor allianceZoneShooting =  new RGBWColor(0, 255, 0, 255); //green 
    public static final RGBWColor climbPrepared =  new RGBWColor(220, 0, 180, 255); //magenta
    public static final RGBWColor climbing =  new RGBWColor(195, 140, 235, 255); //lilac
    public static final RGBWColor idling = new RGBWColor(200, 255, 0, 255); //lime
    public static final RGBWColor stopped =  new RGBWColor(255, 0, 0, 255); //red
}
