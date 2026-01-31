package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

// import frc.robot.commands.RevShooterGoTheOtherWay;
import frc.robot.commands.ShooterWithSparkGO;
import frc.robot.commands.intakeGo;
// import frc.robot.commands.intakeGoTheOtherWay;
// import frc.robot.commands.shootGo;
// import frc.robot.commands.shooterGoTheOtherWay;

import frc.robot.subsystems.ShooterUsingRev;
import frc.robot.subsystems.theActualShooter;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.Limelight2;
// import frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private static final double DEADBAND = 0.08;
    private  double speed = OperatorConstants.kSpeed;
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.Velocity);
    
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController c = new CommandXboxController(OperatorConstants.kOpControllerPort);
  // private final shooter shootter = new shooter();
  private final theActualShooter intakeee = new theActualShooter();
  private final ShooterUsingRev shooter2 = new ShooterUsingRev();
 // private final limelight limee = new limelight();
  private final limelight limer = new limelight();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(()-> {

                double x =
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), DEADBAND);
                double y =
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), DEADBAND);
                double rot =
                    -MathUtil.applyDeadband(m_driverController.getRightX(), DEADBAND);

                return drive
                    .withVelocityX(x * MaxSpeed)
                    .withVelocityY(y * MaxSpeed)
                    .withRotationalRate(rot * MaxAngularRate);
            })
        );

        m_driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> {

                double x =
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), DEADBAND);
                double y =
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), DEADBAND);
                double rot =
                    -MathUtil.applyDeadband(m_driverController.getRightX(), DEADBAND);

                return robotCentric
                    .withVelocityX(x * MaxSpeed) // multiply this by slew limiter
                    .withVelocityY(y * MaxSpeed)
                    .withRotationalRate(rot * MaxAngularRate);
            }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_driverController.a().whileTrue(
            drivetrain.applyRequest(() -> brake)
        );

        m_driverController.b().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(
                    new Rotation2d(
                        -m_driverController.getLeftY(),
                        -m_driverController.getLeftX()
                    )
                )
            )
        );
        m_driverController.leftBumper().whileTrue(new InstantCommand(()-> speed = OperatorConstants.fastSpeed));
        m_driverController.leftTrigger().whileTrue(new InstantCommand(()-> speed = OperatorConstants.slowSpeed));


        m_driverController.back().and(m_driverController.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        m_driverController.back().and(m_driverController.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        m_driverController.start().and(m_driverController.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        m_driverController.start().and(m_driverController.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        m_driverController.leftBumper()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


      
       c.y().whileTrue(new intakeGo(intakeee));// test this one
       c.leftBumper().whileTrue(new ShooterWithSparkGO(shooter2));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

     public double speedScale(){
      if(m_driverController.leftBumper().getAsBoolean())
      return Constants.OperatorConstants.fastSpeed;
        if(m_driverController.leftTrigger().getAsBoolean())
      return Constants.OperatorConstants.slowSpeed;

      return Constants.OperatorConstants.normalSpeed;
     }

    public Command getAutonomousCommand() {

        final var idle = new SwerveRequest.Idle();

        return Commands.sequence(
            drivetrain.runOnce(() ->
                drivetrain.seedFieldCentric(Rotation2d.kZero)
            ),

            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5 * MaxSpeed)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            ).withTimeout(5.0),

            drivetrain.applyRequest(() -> idle)
        );
    }
}