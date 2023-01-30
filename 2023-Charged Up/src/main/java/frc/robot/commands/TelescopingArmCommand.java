package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.TelescopingArmSubsystem;

public class TelescopingArmCommand extends CommandBase { 

    private int _state;

    public TelescopingArmCommand() /* TAKE IN AN ENUM LATER */ {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TelescopingArmSubsystem.setArmLength(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (OI.getOperator().getAButton()) {
            _state = 0;
        }        

        if (OI.getOperator().getBButton()) {
            _state = 1;
        }

        if (OI.getOperator().getXButton()) {
            _state = 2;
        }

        switch (_state) {
            case 0:
                TelescopingArmSubsystem.setArmLength(Constants.minArmLength);
                break;
            case 1:
                TelescopingArmSubsystem.setArmLength(Constants.midArmLength);
                break;
            case 2:
                TelescopingArmSubsystem.setArmLength(Constants.maxArmLength);
                break;
        }
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