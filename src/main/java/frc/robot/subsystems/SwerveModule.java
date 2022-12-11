// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS



/**
 * https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777270385
 */
public class SwerveModule{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANCoder cANCoder;
    final private CANSparkMax driveSparkMax;
    final private CANSparkMax turnSparkMax;

    final private RelativeEncoder driveEncoder;
    final private RelativeEncoder turnEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    final private String module_id;

    private double location_x = 0.0, location_y = 0.0;

    private double driveGearRatio = 8.14; // MK4i: 8.14 or 6.75 or 6.12
    private double turnGearRatio = 150./7.;

    private double base_angle;

    private double maxTurnSpeed = 0.3;
    private double maxDriveSpeed = 0.2;
    private double turnIntegratorRange = 0.01;

    private boolean enable_turn = true;
    private boolean enable_drive = true;

    private double turnVel = 0.0;

    private PIDController angle_pid;

    private final NetworkTable swerve_table;
    private final NetworkTableEntry turn_setpoint;
    private final NetworkTableEntry drive_setpoint;
    private final NetworkTableEntry turn_angle;
    private final NetworkTableEntry drive_velocity;

    public SwerveModule(String module_id, int cancoder_id, int drive_motor_id, int turn_motor_id, double base_angle){
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        this.module_id = module_id;
        this.base_angle = base_angle;

        cANCoder = new CANCoder(cancoder_id);
        driveSparkMax = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(turn_motor_id, MotorType.kBrushless);

        driveSparkMax.restoreFactoryDefaults();
        driveSparkMax.setInverted(false);
        driveSparkMax.setIdleMode(IdleMode.kCoast);
        driveSparkMax.burnFlash();


        turnSparkMax.restoreFactoryDefaults();
        turnSparkMax.setInverted(false);
        turnSparkMax.setIdleMode(IdleMode.kCoast);
        turnSparkMax.burnFlash();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveEncoder = driveSparkMax.getEncoder();

        turnEncoder = turnSparkMax.getEncoder();

        angle_pid = new PIDController(0.007500, 0.0200, 0.0000);
        angle_pid.enableContinuousInput(0.0, 360);

        angle_pid.setTolerance(5.0, 5.0);
        angle_pid.setIntegratorRange(-turnIntegratorRange, turnIntegratorRange);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        swerve_table = inst.getTable("swerve_chassis").getSubTable("module_" + module_id);

        swerve_table.getEntry("module_id").setString(module_id);
        turn_setpoint = swerve_table.getEntry("turn_setpoint");
        turn_setpoint.setDouble(0.0);
        drive_setpoint = swerve_table.getEntry("drive_setpoint");
        drive_setpoint.setDouble(0.0);
        turn_angle = swerve_table.getEntry("turn_angle");
        turn_angle.setDouble(0.0);
        drive_velocity = swerve_table.getEntry("drive_setpoint");
        drive_velocity.setDouble(0.0);
            
        //System.out.printf("Swerve Module Created\n");
    }

    public void periodic() {
        double startTime = System.currentTimeMillis()/1000.0;
        double ang = getCancoderAbsPosition();
        turn_angle.setDouble(ang);

        double ts = turn_setpoint.getDouble(0.0);

        ts = computeAdjustedAngle(ts);
        double c1 = System.currentTimeMillis()/1000.0;
        
        angle_pid.setSetpoint(ts);    
        turnVel = angle_pid.calculate(getCancoderAbsPosition());
        double c2 = System.currentTimeMillis()/1000.0;
        setTurnSpeed(-turnVel);

        double ds = drive_setpoint.getDouble(0.0);
        double c3 = System.currentTimeMillis()/1000.0;
        setDriveSpeed(ds);

        double endTime = System.currentTimeMillis()/1000.0;
        //System.out.printf("SM Periodic %s: ST=%.3f ET=%.3f %.3f %.5f %.3f\n",module_id,startTime,endTime-startTime,c1-startTime,c2-startTime,c3-startTime);

    }

    //public void addToHomeAngle(double angle){
    //    base_angle += angle;
    //}


    private double clipAngle(double angle){ 
        double out_angle = angle;
        out_angle = 360.0*((out_angle/360.0) - Math.floor(out_angle/360.0));
        return out_angle;
    }

    private double computeAdjustedAngle(double angle){ 
        double out_angle =  base_angle+angle;
        out_angle = 360.0*((out_angle/360.0) - Math.floor(out_angle/360.0));
        return out_angle;
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setMaxTurnSpeed(double value) {
        maxTurnSpeed = value;
    }

    public double getMaxTurnSpeed() {
        return maxTurnSpeed;
    }

    public void setMaxDriveSpeed(double value) {
        maxDriveSpeed = value;
    }

    public double getMaxDriveSpeed() {
        return maxDriveSpeed;
    }

    public void setAngle(double value) {
        value = clipAngle(value);
        turn_setpoint.setDouble(value);
    }

    public double getAngle() {
        return turn_setpoint.getDouble(0.0);
    }

    public void setVelocity(double value) {
        drive_setpoint.setDouble(value);

    }

    public void getVelocity() {
        drive_setpoint.getDouble(0.0);
    }

    private void setTurnSpeed(double velocity) {
        if (velocity > maxTurnSpeed) {
            velocity = maxTurnSpeed;
        }
        if (velocity < -maxTurnSpeed) {
            velocity = -maxTurnSpeed;
        }
        if (enable_turn) {
            turnSparkMax.set(velocity);
        }
    }

    private void setDriveSpeed(double velocity) {
        if (velocity > maxDriveSpeed) {
            velocity = maxDriveSpeed;
        }
        if (velocity < -maxDriveSpeed) {
            velocity = -maxDriveSpeed;
        }
        if (enable_drive) {
            driveSparkMax.set(velocity);
        }
    }

    public double getCancoderAbsPosition() {
        return cANCoder.getAbsolutePosition();
    }

    double getTurnMotorValue() {
        return turnSparkMax.get();
    }

    double getDriveMotorValue() {
        return driveSparkMax.get();
    }


}
