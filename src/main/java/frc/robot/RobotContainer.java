// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
	private final SendableChooser<Command> autoChooser;
	// The robot's subsystems and commands are defined here...
	private SwerveDriveSubsystem m_swerve;

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	// private ElevatorSubsystem elevator;
// private final Joystick m_driverController = new Joystick(0);
	// private final Button robotCentric = new m_driverController.getAsBoolean();
	// SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		m_swerve = new SwerveDriveSubsystem();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Mode", autoChooser);
		// elevator = new ElevatorSubsystem(20);
		// Command moveElevatorToTop = elevator.goToSetpointCommand(50.0); // Example
		// target height
		SwerveInputStream driveStream = SwerveInputStream.of(m_swerve.getSwerveDrive(),
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX())
		.withControllerRotationAxis(() -> -m_driverController.getRightX())
		.deadband(0.01)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

SwerveInputStream driveDirectAngle = driveStream.copy()
        .withControllerHeadingAxis(
            () -> -m_driverController.getRightX(), // inverted
            () -> m_driverController.getRightY()
        )
        .headingWhile(true);


		m_swerve.setDefaultCommand(
				m_swerve.driveCommand(driveStream) // or driveDirectAngle
		);

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 * 
	 */
	private void configureBindings() {

		m_driverController.x().onTrue((Commands.runOnce(m_swerve::zeroGyro)));

	}

	public Command getAutonomousCommand() {
		// This method loads the auto when it is called, however, it is recommended
		// to first load your paths/autos when code starts, then return the
		// pre-loaded auto/path
		// return new PathPlannerAuto("Example Auto");
		return autoChooser.getSelected();
	}

}
