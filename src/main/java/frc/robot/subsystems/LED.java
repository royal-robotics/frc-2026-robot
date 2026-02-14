package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    
    public final AddressableLED led = new AddressableLED(0);
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(big number here);

    public LED () {
        led.setLength(ledBuffer.getLength());
        led.start();
    }
   
    public void periodic (){
        led.setData(ledBuffer);
    }
}
