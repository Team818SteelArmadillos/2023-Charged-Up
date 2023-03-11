package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
    
    private LEDSubsystem ledSubsystem;

    public LEDCommand(LEDSubsystem sub) {
      ledSubsystem = sub;
    }

    @Override
    public void initialize() {
      ledSubsystem.setLEDs( OI.getOperator().x().getAsBoolean() );
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // not sure what to put here, will update
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
  }
}