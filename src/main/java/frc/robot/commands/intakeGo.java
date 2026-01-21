package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeGo extends Command{
    intake in;
    public intakeGo(intake in){
        this.in = in;
        addRequirements(in);
    }
 @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    in.go();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    in.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
