package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers;

public class SwerveDriveSubsystem extends SubsystemBase {

	SwerveDrive swerveDrive;

	public final Field2d m_field = new Field2d();
	public Pigeon2Swerve pigeon = new Pigeon2Swerve(20);

	public SwerveDriveSubsystem() {
		// swerveDrive.setCosineCompensator(false); // Disables cosine compensation for
		// simulations since it causes discrepancies not seen in real life.
		// swerveDrive.setGyroOffset(new Rotation3d(0,0,-154.69));
		Limelight.registerDevice("limelight");
		double maximumSpeed = Constants.maximumSpeed;
		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
			SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

			// swerveDrive.setChassisDiscretization(true, .02);
			swerveDrive.setHeadingCorrection(true);
			swerveDrive.setCosineCompensator(true);
			// swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"), Timer.getFPGATimestamp()); 
			LimelightHelpers.getBotPoseEstimate_wpiBlue(getName());
			// NOTE FROM AARON: addVisionMeasurement should be called in the periodic function!!!
			// Use LimelightHelpers.getBotPoseEstimate_wpiBlue instead since it returns a PoseEstimate (Which includes pose, timestamp, and latency).
			// You can see an example of this in Limelight.getPoseEstimate (/Limelight/Limelight.java).

			// swerveDrive.pushOffsetsToEncoders();
			swerveDrive.restoreInternalOffset();
			CommandScheduler.getInstance().registerSubsystem(this);

		} catch (IOException e) {

			throw new RuntimeException(e);
		}
		setupPathPlanner();

		


	}

	@SuppressWarnings("rawtypes")
	public Odometry odometry;

	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Command to drive the robot using translative values and heading as a
	 * setpoint.
	 *
	 * @param translationX Translation in the X direction.
	 * @param translationY Translation in the Y direction.
	 * @param headingX     Heading X to calculate angle of the joystick.
	 * @param headingY     Heading Y to calculate angle of the joystick.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
			DoubleSupplier headingY) {
		return run(() -> {

			Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
					translationY.getAsDouble()), 1);

			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
					headingX.getAsDouble(),
					headingY.getAsDouble(),
					swerveDrive.getOdometryHeading().getRadians(),
					Constants.GetmaximumSpeed()));
		});
	}

	/**
	 * Command to drive the robot using translative values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction.
	 * @param translationY     Translation in the Y direction.
	 * @param angularRotationX Rotation of the robot to set
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			swerveDrive.drive(new Translation2d(translationX.getAsDouble() * Constants.GetmaximumSpeed(),
					translationY.getAsDouble() * Constants.GetmaximumSpeed()),
					angularRotationX.getAsDouble() * Constants.GetmaximumangularSpeed(),
					false,
					true);
		});
	}

	public ChassisSpeeds getRelativeSpeeds() {
		return swerveDrive.getRobotVelocity(); // Check exact swervelib API name
	}
	
	public void resetPose(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}
	

	public Pose2d getPose() {
		if (Robot.isSimulation()) {
			return this.m_field.getRobotPose();
		}
		return swerveDrive.getPose();
	}

	private void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
	
		AutoBuilder.configure(
			this::getPose,
			this::resetPose,
			this::getRelativeSpeeds,
	
			(speeds, ff) -> {
				swerveDrive.drive(
					new Translation2d(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond
					),
					speeds.omegaRadiansPerSecond,
					false, // robot relative
					true
				);
			},
	
			// PathFollowing controller
			new PPHolonomicDriveController(
				new PIDConstants(3.3, 0, 0),
				new PIDConstants(Math.PI * 1.6, 0, 0)
			),
	
			// Robot config
			config,
	
			// Should flip for red alliance?
			() -> {
				var alliance = DriverStation.getAlliance();
				return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
			},
	
			// Subsystem
			this
		);
	}
	

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumberArray("Measured States", swervelib.telemetry.SwerveDriveTelemetry.measuredChassisSpeeds);
		SmartDashboard.putNumberArray("SwerveModuleStates", swervelib.telemetry.SwerveDriveTelemetry.measuredStates);
		SmartDashboard.putNumberArray("Vision Estimated Pose", LimelightHelpers.getBotPose("limelight"));

	}

	public Command driveCommand(SwerveInputStream inputStream) {
		return run(() -> {
			driveFieldOriented(inputStream.get());
		});
	}
}
