package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArmSubsystem;

public class TelescopingArmCommand extends CommandBase { 
    
    private int _state;

    public TelescopingArmCommand(int state) {
        _state = state;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TelescopingArmSubsystem.setArmLength(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (_state) {
            case 0: TelescopingArmSubsystem.setArmLength(Constants.armLengths[0]); break;
            case 1: TelescopingArmSubsystem.setArmLength(Constants.armLengths[1]); break;
            case 2: TelescopingArmSubsystem.setArmLength(Constants.armLengths[2]); break;
            case 3: TelescopingArmSubsystem.setArmLength(Constants.armLengths[3]); break;
            case 4: TelescopingArmSubsystem.setArmLength(Constants.armLengths[4]); break;		
            default: break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return TelescopingArmSubsystem.PID.atSetpoint();
    }  
}