package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeNetwork;

public class LimelightCommand extends CommandBase {

  private LimeNetwork limeNetwork;

  public LimelightCommand (LimeNetwork sub) {
    addRequirements(sub);
    limeNetwork = sub;
  }

  @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartDashboard.putNumberArray("botpose 3d", limeNetwork.getRobotPose() );
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