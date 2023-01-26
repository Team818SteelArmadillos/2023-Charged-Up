package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
    
    /* ex
    CANdle candle;
    I2C.Port i2cPort;   
    */

    // This block initializes all the static variables in the class
    static {
    // Declarations of variables should be moved outside any methods    
    CANdle candle;
    I2C.Port i2cPort;
    }

    public static void() { //set parameters for rgb and reorganize code
        // All this code should be moved to the static block for initialization purposes
        CANdle candle = new CANdle(0);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);
        //

        // This should be moved into the command part of the subsystem
        if (oi.getOperator().getAButton()) { 
            candle.setLEDs(166, 77, 255); 
        }
        else if (oi.getOperator().getBButton()) { 
            candle.setLEDs(255, 195, 77);
        }

        // example to interact
        /*
            public static setLEDs(double r, double g, double b) {
                candle.setLEDs(r, g, b);
            }

            in the command class, you could create a method that takes in two booleans
            that represent the a button press and the b button press and then use the if else
            statement above to set the LEDs accordingly
        */

    }

}