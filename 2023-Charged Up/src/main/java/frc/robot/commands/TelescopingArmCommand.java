package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArmSubsystem;

public class TelescopingArmCommand extends CommandBase { 

    enum Telestate {
        FULL_RETRACT,
        LOW_GOAL,
        MID_GOAL,
        HIGH_GOAL
    }
    
    private Telestate _state;

    public TelescopingArmCommand(Telestate state) {
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
            case FULL_RETRACT: TelescopingArmSubsystem.setArmLength(0);
            case LOW_GOAL: TelescopingArmSubsystem.setArmLength(Constants.minArmLength);
            case MID_GOAL: TelescopingArmSubsystem.setArmLength(Constants.midArmLength);
            case HIGH_GOAL: TelescopingArmSubsystem.setArmLength(Constants.maxArmLength);
			
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