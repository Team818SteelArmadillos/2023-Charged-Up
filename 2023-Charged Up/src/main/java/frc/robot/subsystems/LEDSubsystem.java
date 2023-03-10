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
    static boolean toggle;
    
    public LEDSubsystem() { 
        CANdle candle = new CANdle(0);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);
    }

    public void setLEDs(boolean aButtonPressed) {
        if (aButtonPressed) {
            toggle = true;
        }

        if (aButtonPressed && toggle == true) {
            toggle = false;
        }

        if (toggle == true) {
            candle.setLEDs(166, 77, 255);
        }

        else {
            candle.setLEDs(255, 230, 102);
        }
    }

    }

