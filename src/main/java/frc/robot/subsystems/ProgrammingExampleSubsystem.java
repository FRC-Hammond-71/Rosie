package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// MOST USEFUL IMPORTS ARE UNDER com.revrobotics.spark or edu.wpi.first.wpilibj

public class ProgrammingExampleSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;

/*  
        This is the programming language Java! This means that:

        When making a variable, it NEEDS a type. (int, string, int[] (int array), etc)

        When making a function, you should probably make it public or private.

        Make sure the variables you reference actually have values.
        (please dont NullPointerException it would make me sad)

        other than that go off, be gay do crime have fun 
        

        just dont uh break SwerveDriveSubsystem :)
        -kellon
*/

    // Constructor
    public ProgrammingExampleSubsystem() {
        intakeMotor = new SparkMax(30, MotorType.kBrushless);
    }

    // Example intake function
    public void intake() {
        intakeMotor.set(-0.5);
    }

    // Example score function
    public void score() {
        intakeMotor.set(0.5);
    }

    public void stop() {
        intakeMotor.set(0);
    }


}
