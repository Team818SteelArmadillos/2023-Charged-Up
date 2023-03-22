package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.I2C;

public class LEDSubsystem extends SubsystemBase {  
    I2C.Port i2cPort;
    CANdle candle;
    CANdleConfiguration config;
    boolean toggle;
    
    public LEDSubsystem() {
        candle = new CANdle(Constants.CANDLE_CAN_ID);

        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);  
                
        candle.setLEDs(0, 102, 255); 

        toggle = true;
    }

    public void setLEDsPurple() {        
        candle.setLEDs(153, 0, 204);
    }

    public void setLEDsYellow() {
        candle.setLEDs(255, 255, 0);
    }

    public void toggle() {
        if (toggle) {
            setLEDsYellow();
        } else {
            setLEDsPurple();
        }

        toggle = !toggle;
    }
} 

