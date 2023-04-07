package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

public class ClawWheelCommand extends CommandBase {

  int _state;
  private ClawSubsystem clawWheelsSubsystem;
  private int rumble_counter;
  
  public ClawWheelCommand(int state, ClawSubsystem sub) {
    _state = state;
    clawWheelsSubsystem = sub;
    rumble_counter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!clawWheelsSubsystem.inCubeMode()) {
      switch (_state) {
        case 0: clawWheelsSubsystem.setIntakeSpeed(Constants.CONE_IN_SPEED); break; //Cone in
        case 1: clawWheelsSubsystem.setIntakeSpeed(Constants.CONE_OUT_SPEED); break; //Cone out
      }
    } else {
      switch (_state) {
        case 0: clawWheelsSubsystem.setIntakeSpeed(Constants.CUBE_IN_SPEED); break; //Cube in
        case 1: clawWheelsSubsystem.setIntakeSpeed(Constants.CUBE_OUT_SPEED); break; //Cube out
      }
    }

    // if (clawWheelsSubsystem.getMotorCurrent() >= Constants.CLAW_RUMBLE_AMP_THRESHOLD) {
    //   rumble_counter = 0;
    // } else {
    //   rumble_counter++;
    // }

    // if (rumble_counter <= 20) {

    //   OI.getDriver().setRumble(RumbleType.kBothRumble, 0.5);
    //   OI.getXOperator().setRumble(RumbleType.kBothRumble, 0.5);
    // } else {
    //   rumble_counter = 21;
    //   OI.getDriver().setRumble(RumbleType.kBothRumble, 0.0);
    //   OI.getXOperator().setRumble(RumbleType.kBothRumble, 0.0);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawWheelsSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }
     
}
