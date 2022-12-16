package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    //private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 10, 1, 2, 303.0 + 45); this angle is 180 out of phase
    //private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 11, 3, 4, 358 + 315);
    //private final SwerveModule m_swerveModule_br = new SwerveModule("br", 12, 5, 6, 242.0 + 135);
    //private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 13, 7, 8, 187.0 + 225);

    // All Wheels Reversed
    //private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 10, 1, 2, 35.0 + 45);
    //private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 11, 3, 4, 359.0 + 315);
    //private final SwerveModule m_swerveModule_br = new SwerveModule("br", 12, 5, 6, 225.0 + 135);
    //private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 13, 7, 8, 187.0 + 225);

    //private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 10, 1, 2, 215.0 + 45);
    //private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 11, 3, 4, 179.0 + 315);
    //private final SwerveModule m_swerveModule_br = new SwerveModule("br", 12, 5, 6, 45.0 + 135);
    //private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 13, 7, 8, 7.0 + 225);
    private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 10, 1, 2, 285.0 + 45);
    private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 11, 3, 4, 179.0 + 315);
    private final SwerveModule m_swerveModule_br = new SwerveModule("br", 12, 5, 6, 30.0 + 135);
    private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 13, 7, 8, 7.0 + 225);

    // Locations for the swerve drive modules relative to the robot center.
    //Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    //Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    //Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    //Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    //Translation2d m_frontLeftLocation = new Translation2d(0.4826, 0.4826);
    //Translation2d m_frontRightLocation = new Translation2d(0.4826, -0.4826);
    //Translation2d m_backLeftLocation = new Translation2d(-0.4826, 0.4826);
    //Translation2d m_backRightLocation = new Translation2d(-0.4826, -0.4826);
    Translation2d m_frontLeftLocation = new Translation2d(0.3683, 0.3683);
    Translation2d m_frontRightLocation = new Translation2d(0.3683, -0.3683);
    Translation2d m_backLeftLocation = new Translation2d(-0.3683, 0.3683);
    Translation2d m_backRightLocation = new Translation2d(-0.3683, -0.3683);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
            m_backLeftLocation, m_backRightLocation);

    final private NetworkTable swerve_table;
    final private NetworkTableEntry x_velocity;
    final private NetworkTableEntry y_velocity;
    final private NetworkTableEntry r_velocity;

    final private NetworkTableEntry drive_enabled;

    public SwerveDrive() {
        swerve_table = NetworkTableInstance.getDefault().getTable("swerve_chassis");
        x_velocity = swerve_table.getEntry("x_velocity");
        x_velocity.setDouble(0.0);
        y_velocity = swerve_table.getEntry("y_velocity");
        y_velocity.setDouble(0.0);
        r_velocity = swerve_table.getEntry("r_velocity");
        r_velocity.setDouble(0.0);
        drive_enabled = swerve_table.getEntry("enabled");
        drive_enabled.setBoolean(true);
    }

    public void setChassisSpeeds(double x_vel, double y_vel, double r_vel) {
        if (Math.abs(x_vel) < 0.1) {
            x_velocity.setDouble(0.0);
        } else {
            x_velocity.setDouble(x_vel);
        }
        if (Math.abs(y_vel) < 0.1) {
            y_velocity.setDouble(0.0);
        } else {
            y_velocity.setDouble(y_vel);
        }
        if (Math.abs(r_vel) < 0.1) {
            r_velocity.setDouble(0.0);
        } else {
            r_velocity.setDouble(r_vel);
        }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        double startTime = System.currentTimeMillis()/1000.0;

        super.periodic();

        double dx = x_velocity.getDouble(0);
        double dy = y_velocity.getDouble(0);
        double rads_per_sec = r_velocity.getDouble(0);
        double c1 = System.currentTimeMillis()/1000.0;

        ChassisSpeeds speeds = new ChassisSpeeds(dx, dy, rads_per_sec);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        double c2 = System.currentTimeMillis()/1000.0;

        // fl_angle.setDouble( states[0].angle.getDegrees() );
        // fr_angle.setDouble( states[1].angle.getDegrees() );
        // bl_angle.setDouble( states[2].angle.getDegrees() );
        // br_angle.setDouble( states[3].angle.getDegrees() );

        // fl_speed.setDouble( states[0].speedMetersPerSecond );
        // fr_speed.setDouble( states[1].speedMetersPerSecond );
        // bl_speed.setDouble( states[2].speedMetersPerSecond );
        // br_speed.setDouble( states[3].speedMetersPerSecond );

        double c3 = startTime;
        double c4 = startTime;
        if (drive_enabled.getBoolean(true)) {
            m_swerveModule_fl.setAngle(states[0].angle.getDegrees());
            m_swerveModule_fr.setAngle(states[1].angle.getDegrees());
            m_swerveModule_bl.setAngle(states[2].angle.getDegrees());
            m_swerveModule_br.setAngle(states[3].angle.getDegrees());

            m_swerveModule_fl.setVelocity(states[0].speedMetersPerSecond);
            m_swerveModule_fr.setVelocity(states[1].speedMetersPerSecond);
            m_swerveModule_bl.setVelocity(states[2].speedMetersPerSecond);
            m_swerveModule_br.setVelocity(states[3].speedMetersPerSecond);
            c3 = System.currentTimeMillis()/1000.0;

            m_swerveModule_bl.periodic();
            m_swerveModule_br.periodic();
            c4 = System.currentTimeMillis()/1000.0;
            m_swerveModule_fl.periodic();
            m_swerveModule_fr.periodic();
        }
        double endTime = System.currentTimeMillis()/1000.0;
        //System.out.printf("SD Periodic: ST=%.3f  ET=%.3f EL=%.3f %.3f %.3f %.3f %.3f\n",startTime,endTime,endTime-startTime,c1-startTime,c2-startTime,c3-startTime,c4-startTime);
        //System.out.flush();
    }
}
