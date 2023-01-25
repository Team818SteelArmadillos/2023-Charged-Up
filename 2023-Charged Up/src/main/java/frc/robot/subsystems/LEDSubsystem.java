package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {


    static {
    CANdle candle;
    I2C.Port i2cPort;
    }

    public static void() { //set parameters for rgb and reorganize code
        CANdle candle = new CANdle(0);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);

        if (oi.getOperator().getAButton()) { 
            candle.setLEDs(166, 77, 255); 
        }
        else if (oi.getOperator().getBButton()) { 
            candle.setLEDs(255, 195, 77);
        }

}
}
