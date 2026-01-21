package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterUsingRev;

public class ShooterWithSparkGO extends Command{
    private ShooterUsingRev s;
     public ShooterWithSparkGO(ShooterUsingRev s){
        this.s = s;
        addRequirements(s);
    }
 @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s.spin();;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
