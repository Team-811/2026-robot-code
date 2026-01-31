// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class theActualShooter extends SubsystemBase{

//     TalonFX intakee;
//     private final double cH = 0;
//     private final double H = 0; // change to measured values later
//     private final VelocityVoltage velo = new VelocityVoltage(0);
//     private final double targetV;
//     public theActualShooter(){
//         intakee = new TalonFX(54);

//           Slot0Configs slot0 = new Slot0Configs();
//         slot0.kP = 0.12;
//         slot0.kI = 0.0;
//         slot0.kD = 0.0;
//         slot0.kV = 0.0; // feedforward

//         intakee.getConfigurator().apply(slot0);
//             double ty = NetworkTableInstance.getDefault()
//         .getTable("limelight-lime")
//         .getEntry("ty")
//         .getDouble(0);

//         double distance = (H - cH)/Math.tan(Math.toRadians(ty));
//         targetV = getSpeed(distance);

//         intakee.getConfigurator().apply(slot0);
//     }
//         public void go(){
//         // intakee.set(0.25);
//         intakee.setControl(velo.withVelocity(targetV));
  
//     }
//     public double getSpeed(double distance){
//         if(distance < 0) // change to tested value
//             return 0;
//         else if(distance < 0)
//             return 0; //if need add more if statments
//         else 
//             return 0;
//     }
//     // public void goTheOtherWay(){
//     //     intakee.set(-1);
    
//     // }
//     public void stopIntake(){
//         intakee.set(0);
  
//     }
//     @Override
//     public void periodic(){

//     }
// }
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class theActualShooter extends SubsystemBase {

    private final TalonFX intakee = new TalonFX(54);
    private final VelocityVoltage velo = new VelocityVoltage(0);

    /* ---------------- LIMELIGHT CONSTANTS ---------------- */
    private static final String LIMELIGHT_NAME = "limelight-lime";

    private static final double LIMELIGHT_HEIGHT = 0.80; // meters
    private static final double TARGET_HEIGHT = 2.10;    // meters
    private static final double LIMELIGHT_ANGLE = Units.degreesToRadians(25);

    /* ---------------- SHOOTER TUNING ---------------- */
    private static final double VELOCITY_TOLERANCE = 1.5; // rps

    private final InterpolatingDoubleTreeMap distanceToRPM =
            new InterpolatingDoubleTreeMap();

    private double targetRPM = 0;

    public theActualShooter() {

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0.12;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12; // tune later

        intakee.getConfigurator().apply(slot0);

        /* --------- DISTANCE → RPM MAP (TUNE THESE) --------- */
        distanceToRPM.put(2.0, 3200.0);
        distanceToRPM.put(2.5, 3500.0);
        distanceToRPM.put(3.0, 3800.0);
        distanceToRPM.put(3.5, 4150.0);
    }

    /* ================== MAIN SHOOT METHOD ================== */
    public void go() {
        double distance = getDistanceMeters();
        if (distance < 0) return;

        targetRPM = distanceToRPM.get(distance);
        intakee.setControl(
            velo.withVelocity(targetRPM / 60.0) // RPM → RPS
        );
    }

    /* ================== LIMELIGHT DISTANCE ================== */
    private double getDistanceMeters() {
        var table = NetworkTableInstance.getDefault()
                .getTable(LIMELIGHT_NAME);

        boolean hasTarget = table.getEntry("tv").getDouble(0) == 1;
        if (!hasTarget) return -1;

        double ty = table.getEntry("ty").getDouble(0);

        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                Math.tan(LIMELIGHT_ANGLE + Units.degreesToRadians(ty));
    }

    /* ================== STATUS METHODS ================== */
    public boolean isAtSpeed() {
        double currentRPS =
                intakee.getVelocity().getValueAsDouble();
        double targetRPS = targetRPM / 60.0;

        return Math.abs(currentRPS - targetRPS) < VELOCITY_TOLERANCE;
    }

    public void stopIntake() {
        intakee.stopMotor();
    }

    @Override
    public void periodic() {
        // Optional: telemetry to Shuffleboard
    }
}
