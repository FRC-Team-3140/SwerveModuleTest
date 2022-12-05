# Installation

CANCoder

* Download the CTRE Phenox Library (tested 5.21) [link](https://store.ctr-electronics.com/software/)
* Copy the java libraries "mavin" to the wpi library dir wpilib/2022
  * Add to your java project: https://phoenix-documentation.readthedocs.io/en/latest/ch05a_CppJava.html 
* Copy the RobotBuilder to the RobotBuilder extensions directory.
* Open Robot Builder.  
  * Create a swerve module subsystem.
  * Add a SparkMax controller for the turn motor
  * Add a SparkMax controller for the drive motor
  * Add a Cancoder

Add SparkMaxCan Robot Builder Extension


# Robot Builder

# Coding the robot

```
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("cancoder_abs_position", this::getCancoderAbsPosition, null);
        builder.addDoubleProperty("cancoder_id", this::getCancoderDeviceId, null);
        builder.addDoubleProperty("cancoder_position", this::getCancoderPosition, null);
        builder.addDoubleProperty("cancoder_velocity", this::getCancoderVelocity, null);
        builder.addDoubleProperty("turn_value", this::getTurnMotorValue, null);
        builder.addDoubleProperty("drive_value", this::getDriveMotorValue, null);
    }
```

# Shuffleboard