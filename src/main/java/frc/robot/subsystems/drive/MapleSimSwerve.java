package frc.robot.subsystems.drive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import java.util.function.DoubleSupplier;

public class MapleSimSwerve {

    private final SelfControlledSwerveDriveSimulation sim;
    private final Field2d field;
	SwerveDrive swerveDrive;

    public MapleSimSwerve() {
        DriveTrainSimulationConfig config =
                DriveTrainSimulationConfig.Default();

        sim = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(
                        config,
                        new Pose2d(0, 0, new Rotation2d())
                )
        );

        SimulatedArena.getInstance()
                .addDriveTrainSimulation(sim.getDriveTrainSimulation());

        field = new Field2d();
        SmartDashboard.putData("Simulation Field", field);
    }

    /** Update sim + dashboard */
    public void update() {
        Pose2d pose = sim.getDriveTrainSimulation().getSimulatedDriveTrainPose();
        field.setRobotPose(pose);
    }

    public Pose2d getPose() {
        return sim.getDriveTrainSimulation().getSimulatedDriveTrainPose();
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    