package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.I2C;

public class LEDSubsystem extends SubsystemBase {  
    static I2C.Port i2cPort;
    static CANdle candle;
    static CANdleConfiguration config;
    static boolean debounce = true;
    
    public LEDSubsystem() { 
        candle = new CANdle(18);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);
    }

    public void setLEDsViolet() {        
        candle.setLEDs(166, 77, 255);
    }

    public void setLEDsYellow() {
        candle.setLEDs(255, 255, 0);
    }

    public void toggle() {

        if (debounce) {
            setLEDsViolet();
            debounce = false;
        } else {
            setLEDsYellow();
            debounce = true;
        }

    }

}

