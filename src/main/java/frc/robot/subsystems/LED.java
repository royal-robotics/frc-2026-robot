package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Random;

import com.ctre.phoenix6.configs.LEDConfigs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    
    public final AddressableLED spindexerled = new AddressableLED(2);
    public final AddressableLEDBuffer spindexerledBuffer = new AddressableLEDBuffer(66);

    public final LEDPattern rainbowPattern = LEDPattern.rainbow(255,255);
    public final LEDPattern rainbowScroll = rainbowPattern.scrollAtAbsoluteSpeed(FeetPerSecond.of(2), Feet.of(1.0/6.0));
    public final LEDPattern dimRainbowPattern = rainbowScroll.atBrightness(Percent.of(20));
    public final LEDPattern fastrainbowScroll = rainbowPattern.scrollAtAbsoluteSpeed(FeetPerSecond.of(20), Feet.of(1.0/6.0));
    public final LEDPattern fastdimRainbowPattern = fastrainbowScroll.atBrightness(Percent.of(20));

    public final LEDPattern greenbright = LEDPattern.solid(Color.kGreen);
    public final LEDPattern green = greenbright.atBrightness(Percent.of(15));
    public final LEDPattern greenbreath10 = green.breathe(Seconds.of(5));
    public final LEDPattern greenblink = green.blink(Seconds.of (0.5),Seconds.of(0.5));
    public final LEDPattern rapidgreenblink = green.blink(Seconds.of (0.45),Seconds.of(0.05));

    public final LEDPattern lightgreen = LEDPattern.solid(Color.kGreenYellow);
    public final LEDPattern lightgreenbreath10 = lightgreen.breathe(Seconds.of(10));
    
    private final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern dimyellow = yellow.atBrightness(Percent.of(10));
    private final LEDPattern yellowbreath10 = dimyellow.breathe(Seconds.of(5));
    private final LEDPattern yellowblink = dimyellow.blink(Seconds.of (0.45),Seconds.of(0.05));

    private final LEDPattern purple = LEDPattern.solid(Color.kPurple);
    private final LEDPattern dimpurple = purple.atBrightness(Percent.of(10));
    private final LEDPattern purplebreath = dimpurple.breathe(Seconds.of(5));
    private final LEDPattern purpleblink = dimpurple.blink(Seconds.of (0.5),Seconds.of(0.5));
    private final LEDPattern slowpurpleblink = dimpurple.blink(Seconds.of (4),Seconds.of(1));

    private final LEDPattern orange = LEDPattern.solid(Color.kOrange);
    private final LEDPattern dimorange = orange.atBrightness(Percent.of(15));
    private final LEDPattern orangebreath = dimorange.breathe(Seconds.of(5));
    private final LEDPattern orangeblink = dimorange.blink(Seconds.of (0.5),Seconds.of(0.5));
    private final LEDPattern sloworangeblink = dimorange.blink(Seconds.of (4),Seconds.of(1));

    private final LEDPattern pink = LEDPattern.solid(Color.kDeepPink);
    private final LEDPattern dimpink = pink.atBrightness(Percent.of(10));
    private final LEDPattern pinkbreath = dimpink.breathe(Seconds.of(5));
    private final LEDPattern pinkblink = dimpink.blink(Seconds.of (0.5),Seconds.of(0.5));

    private final LEDPattern white = LEDPattern.solid(Color.kFloralWhite);
    private final LEDPattern dimwhite = white.atBrightness(Percent.of(10));
    private final LEDPattern whitekbreath = dimwhite.breathe(Seconds.of(5));
    private final LEDPattern whiteblink = dimwhite.blink(Seconds.of (0.5),Seconds.of(0.5));

    private final LEDPattern teal = LEDPattern.solid(Color.kTeal);
    private final LEDPattern dimteal = teal.atBrightness(Percent.of(10));
    private final LEDPattern tealblink = dimteal.blink(Seconds.of (0.5),Seconds.of(0.5));
    private final LEDPattern slowtealblink = dimteal.blink(Seconds.of (4),Seconds.of(1));
    private final LEDPattern tealbreath = dimteal.breathe(Seconds.of(5));

    private final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern dimblue = blue.atBrightness(Percent.of(10));
    private final LEDPattern rapidblueblink = dimblue.blink(Seconds.of (0.45),Seconds.of(0.05));

    private final LEDPattern red = LEDPattern.solid(Color.kRed);
    private final LEDPattern dimred = red.atBrightness(Percent.of(10));
    private final LEDPattern rapidredblink = dimred.blink(Seconds.of (0.45),Seconds.of(0.05));
    private final LEDPattern redblink = dimred.blink(Seconds.of (0.9),Seconds.of(0.1));

    public boolean blueWinAuto  = false;
    public boolean weareblue = false;

    public boolean Climbing = true;

    public  LEDPattern patternToApply = dimRainbowPattern;

    public boolean hasAllinace = false;
    public boolean hasWinner = false;

    public LED () {
        spindexerled.setLength(spindexerledBuffer.getLength());
        patternToApply.applyTo(spindexerledBuffer);
        spindexerled.start();
        setDefaultCommand(shiftLogic());
    }
   
    public void periodic (){
        spindexerled.setData(spindexerledBuffer);
    }

    public void ClimberStatus(boolean toggle){
        Climbing = toggle;
    }


    public Command Yellow () {
        return run ( ()-> yellow.applyTo(spindexerledBuffer)).ignoringDisable(true);
    }

    public Command Rainbow () {
        return run(()-> dimRainbowPattern.applyTo(spindexerledBuffer)).ignoringDisable(true);
    }

    public Command FastRainbow () {
        return run(()-> fastdimRainbowPattern.applyTo(spindexerledBuffer)).ignoringDisable(true);
    }

    public Command Purple () {
        return run ( ()-> dimpurple.applyTo(spindexerledBuffer)).ignoringDisable(true);
    }

    public Command shiftLogic() {
        return run(()->{
            if (hasAllinace == false) {
                if(DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get() == Alliance.Red){
                    weareblue = false;
                    hasAllinace = true;
                } else {
                    weareblue = true;
                    hasAllinace = true;
                }
            }
            if (hasWinner == false) {
                String gamedata = DriverStation.getGameSpecificMessage();
                if(gamedata.length() > 0) {
                    switch (gamedata.charAt(0)) {
                        case 'B':
                            blueWinAuto = true;
                            break;
                        case 'R':
                            blueWinAuto = false;
                            break;
                    
                        default:
                            break;
                    }
                    hasWinner = true;
                }
            }
            double matchtime = DriverStation.getMatchTime();
            if ((hasWinner==false)){
                if (matchtime > 135.0&&matchtime <= 140.0) {
                    yellowbreath10.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 130.0&&matchtime <= 135.0) {
                    yellowblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 110.0&&matchtime <= 130.0) {
                    pinkbreath.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 105.0&&matchtime <= 110.0) {
                    pinkblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 85.0&&matchtime <= 105.0) {
                    whitekbreath.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 80.0&&matchtime <= 85.0) {
                    whiteblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 60.0&&matchtime <= 80.0) {
                     pinkbreath.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 55.0&&matchtime <= 60.0) {
                    pinkblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 35.0&&matchtime <= 55.0) {
                   whitekbreath.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 30.0&&matchtime <= 35.0) {
                    whiteblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 10.0&&matchtime <= 30.0) {
                    slowtealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 5.0&&matchtime <= 10.0) {
                    tealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 0.0&&matchtime <= 5.0) {
                    rapidblueblink.applyTo(spindexerledBuffer);
                }
                if (Climbing == true){
                    fastdimRainbowPattern.applyTo(spindexerledBuffer);
                }
            }
            else if ((blueWinAuto == true&&weareblue == true)||(blueWinAuto == false&&weareblue == false)){
                if (matchtime > 135.0&&matchtime <= 140.0) {
                    greenblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 130.0&&matchtime <= 135.0) {
                    rapidgreenblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 110.0&&matchtime <= 130.0) {
                    sloworangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 105.0&&matchtime <= 110.0) {
                    orangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 85.0&&matchtime <= 105.0) {
                    slowpurpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 80.0&&matchtime <= 85.0) {
                    purpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 60.0&&matchtime <= 80.0) {
                    sloworangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 55.0&&matchtime <= 60.0) {
                    orangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 35.0&&matchtime <= 55.0) {
                   slowpurpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 30.0&&matchtime <= 35.0) {
                    purpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 10.0&&matchtime <= 30.0) {
                    slowtealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 5.0&&matchtime <= 10.0) {
                    tealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 0.0&&matchtime <= 5.0) {
                    rapidblueblink.applyTo(spindexerledBuffer);
                }
                if (Climbing == true){
                    fastdimRainbowPattern.applyTo(spindexerledBuffer);
                }
            } else {
                if (matchtime > 135.0&&matchtime <= 140.0) {
                    redblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 130.0&&matchtime <= 135.0) {
                    rapidredblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 110.0&&matchtime <= 130.0) {
                    slowpurpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 105.0&&matchtime <= 110.0) {
                    purpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 85.0&&matchtime <= 105.0) {
                    sloworangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 80.0&&matchtime <= 85.0) {
                    orangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 60.0&&matchtime <= 80.0) {
                    slowpurpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 55.0&&matchtime <= 60.0) {
                    purpleblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 35.0&&matchtime <= 55.0) {
                   sloworangeblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 30.0&&matchtime <= 35.0) {
                    orangeblink.applyTo(spindexerledBuffer);
                }
                 if (matchtime > 10.0&&matchtime <= 30.0) {
                    slowtealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 5.0&&matchtime <= 10.0) {
                    tealblink.applyTo(spindexerledBuffer);
                }
                else if (matchtime > 0.0&&matchtime <= 5.0) {
                    rapidblueblink.applyTo(spindexerledBuffer);
                }
                if (Climbing == true){
                    fastdimRainbowPattern.applyTo(spindexerledBuffer);
                }
            }


        });
    }
}
