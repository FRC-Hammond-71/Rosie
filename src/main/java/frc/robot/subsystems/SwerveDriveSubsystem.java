package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;


import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    double maximumSpeed = Units.feetToMeters(4.5);
    private boolean fieldRelative = true;
    public Pigeon2Swerve pigeon = new Pigeon2Swerve(20);
    

    public SwerveDriveSubsystem(){

        double maximumSpeed = Constants.GetmaximumSpeed();
        try{
             File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
             SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            //  swerveDrive.setChassisDiscretization(true, .02);
             swerveDrive.setHeadingCorrection(false);

        } catch(IOException e){

            throw new RuntimeException(e);
        }
        

    } 

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }


  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()*1000000), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      Constants.GetmaximumSpeed()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * Constants.GetmaximumSpeed(),
                                          -translationY.getAsDouble() * Constants.GetmaximumSpeed()),
                        angularRotationX.getAsDouble() * Constants.GetmaximumangularSpeed(),
                        true,
                        false);
    });
  }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    




    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Chassis Speed", swervelib.telemetry.SwerveDriveTelemetry.measuredChassisSpeeds);
        SmartDashboard.putNumberArray("Swerve Module States", swervelib.telemetry.SwerveDriveTelemetry.measuredStates);
    }


}



