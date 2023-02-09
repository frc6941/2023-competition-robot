package frc.robot.utils;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.patterns.BlinkingPattern;
import org.frcteam6941.led.patterns.ScannerPattern;
import org.frcteam6941.led.patterns.SolidColorPattern;

import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    public static final AddressableLEDPattern CONNECTING = new BlinkingPattern(Color.kRed, 0.1);
    public static final AddressableLEDPattern ALLIANCE_RED = new ScannerPattern(Color.kFirstRed, Color.kBlack, 5);
    public static final AddressableLEDPattern ALLIANCE_BLUE = new ScannerPattern(Color.kFirstBlue, Color.kBlack, 5);

    public static final AddressableLEDPattern GOT_GAMEPIECE = new BlinkingPattern(Color.kAliceBlue, 0.1);
    public static final AddressableLEDPattern SCORE_CONE = new BlinkingPattern(Color.kYellow, 0.1);
    public static final AddressableLEDPattern SCORE_CUBE = new BlinkingPattern(Color.kPurple, 0.1);

    public static final AddressableLEDPattern COMMUTE_CONE = new ScannerPattern(Color.kYellow, 10);
    public static final AddressableLEDPattern COMMUTE_CUBE = new ScannerPattern(Color.kPurple, 10);

    public static final AddressableLEDPattern LOAD_CONE = new SolidColorPattern(Color.kYellow);
    public static final AddressableLEDPattern LOAD_CUBE = new SolidColorPattern(Color.kPurple);

    public static final AddressableLEDPattern MANUAL = new BlinkingPattern(Color.kRed, 0.2);
}
