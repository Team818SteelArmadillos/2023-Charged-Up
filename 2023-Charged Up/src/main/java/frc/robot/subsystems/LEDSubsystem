package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDNumbers.*;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDS extends SubsystemBase {

    CANdle candle;
    Color ballColor1;
    Color ballColor2;
    ColorSensorV3 colorSensor1;
    ColorSensorV3 colorSensor2;
    I2C.Port i2cPort;
    ColorMatch m_colorMatcher = new ColorMatch();

    public LEDS() {
        i2cPort = I2C.Port.kOnboard;
        ColorSensorV3 colorSensor1 = new ColorSensorV3(i2cPort);
        ballColor1 = colorSensor1.getColor();
        ColorSensorV3 colorSensor2 = new ColorSensorV3(i2cPort);
        ballColor2 = colorSensor2.getColor();
        // supposed to report to smart dashboard the color of each of ball will say blue and or red depeding on ball color
        SmartDashboard.putNumber("Red", ballColor1.red);
        SmartDashboard.putNumber("Red", ballColor2.red);
        SmartDashboard.putNumber("Blue", ballColor1.blue);
        SmartDashboard.putNumber("Blue", ballColor2.blue);

        candle = new CANdle(0);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        candle.configAllSettings(config);

        //sets the leds on the canfle itself to lime green
       candle.setLEDs(50, 205, 50, 255, 0, LED_CANdle);

}
}