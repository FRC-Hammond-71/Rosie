package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.imu.Pigeon2Swerve;
import swervelib.SwerveDrive;


import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    private boolean fieldRelative = true;
    public Pigeon2Swerve pigeon = new Pigeon2Swerve(20);
    

    public SwerveDriveSubsystem(){

        double maximumSpeed = Constants.kMaxSpeed;
        try{
             File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
             SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            //  swerveDrive.setChassisDiscretization(true, .02);
             swerveDrive.setHeadingCorrection(true);

        } catch(IOException e){

            throw new RuntimeException(e);
        }
        

    } 


    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                fieldRelative,
                false
            );
        });
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
        swerveDrive.setGyro(new Rotation3d(0,0,0));
    }

    




    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Chassis Speed", swervelib.telemetry.SwerveDriveTelemetry.measuredChassisSpeeds);
        SmartDashboard.putNumberArray("Swerve Module States", swervelib.telemetry.SwerveDriveTelemetry.measuredStates);
    }


}
