package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase{
    TalonFX intakee;
    public intake(){
        intakee = new TalonFX(54);
    }
        public void go(){
        intakee.set(0.25);
  
    }
    // public void goTheOtherWay(){
    //     intakee.set(-1);
    
    // }
    public void stopIntake(){
        intakee.set(0);
  
    }
    @Override
    public void periodic(){

    }
}
