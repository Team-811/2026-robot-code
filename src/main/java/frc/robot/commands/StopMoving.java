package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.limelight;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class StopMoving extends Command{
    limelight lime;
    // private final CommandSwerveDrivetrain drivetrain;
    // private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    //                private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed; // kSpeedAt12Volts desired top speed
           
    //  private final RobotCentric robotCentric = new SwerveRequest.RobotCentric()
    //         .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate*0.1)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

          private SlewRateLimiter slewLimY = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewLimX = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewLimRoteLime = new SlewRateLimiter(1.5);

          public StopMoving(){

          }
              @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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