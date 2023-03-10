package frc.robot.utils;

import org.frcteam6941.led.AddressableLEDPattern;
import org.frcteam6941.led.patterns.AlternatingColorPattern;
import org.frcteam6941.led.patterns.BlinkingPattern;
import org.frcteam6941.led.patterns.ScannerPattern;
import org.frcteam6941.led.patterns.SolidColorPattern;

import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    public static final AddressableLEDPattern CONNECTING = new BlinkingPattern(Color.kRed, 0.2);
    public static final AddressableLEDPattern ESTOP = new SolidColorPattern(Color.kRed);
    public static final AddressableLEDPattern ALLIANCE_RED = new ScannerPattern(Color.kFirstRed, Color.kBlack, 2);
    public static final AddressableLEDPattern ALLIANCE_BLUE = new ScannerPattern(Color.kFirstBlue, Color.kBlack, 2);

    public static final AddressableLEDPattern AUTO_SHOW = new AlternatingColorPattern(
        new Color[] {
            Color.kRed,
            Color.kOrangeRed,
            Color.kYellow,
            Color.kGreen,
            Color.kCyan,
            Color.kDarkBlue,
            Color.kPurple
        }
    );

    public static final AddressableLEDPattern LOAD_CONE = new BlinkingPattern(Color.kYellow, 0.2);
    public static final AddressableLEDPattern LOAD_CUBE = new BlinkingPattern(Color.kPurple, 0.2);
    public static final AddressableLEDPattern HAS_GAMEPIECE = new SolidColorPattern(Color.kGreen);

    public static final AddressableLEDPattern COMMUTE_CONE = new BlinkingPattern(Color.kYellow, 0.5);
    public static final AddressableLEDPattern COMMUTE_CUBE = new BlinkingPattern(Color.kPurple, 0.5);
    public static final AddressableLEDPattern SCORING = new SolidColorPattern(Color.kWhite);
    public static final AddressableLEDPattern SCORING_HAS_VISION = new BlinkingPattern(Color.kWhite, 0.2);

    public static final AddressableLEDPattern MANUAL = new BlinkingPattern(Color.kRed, 0.05);
}
