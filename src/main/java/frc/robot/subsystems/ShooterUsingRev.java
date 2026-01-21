package frc.robot.subsystems;
// import com.revrobotics.spark.CANSparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterUsingRev extends SubsystemBase {
    private SparkFlex shoot;
    public ShooterUsingRev(){
        shoot = new SparkFlex(3, MotorType.kBrushless);
    }
    public void spin(){
        shoot.set(0.25);
    }
    // public void spinTheOtherWay(){
    //     shoot.set(-0.75);
    // }
    public void stopShooter(){
        shoot.set(0);
    }
}
