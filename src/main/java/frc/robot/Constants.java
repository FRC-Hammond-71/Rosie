// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import dev.doglog.DogLog;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final double maximumSpeed = 3;
	public static final double maximumAngleSpeed = 20;


	public static final Distance shooterSideOffset = Inches.of(6.0);

    public static final Transform2d shooterTransform = new Transform2d(Inches.of(0.0), shooterSideOffset, new Rotation2d());

    public static final Pose3d redHubPose = new Pose3d(Inches.of(468.56), Inches.of(158.32), Inches.of(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Inches.of(152.56), Inches.of(158.32),  Inches.of(72.0), new Rotation3d());

    public static final Pose3d redFerryPose = new Pose3d(14.3, 4.02, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPose = new Pose3d(2.1, 4.02, 0, Rotation3d.kZero);

    public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

	public static final Pose3d getHubPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
        DogLog.log("HUB POSE", pose);
        return pose;
    }

	 public static final Pose3d getFerryPose() {
        Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redFerryPose : blueFerryPose;
        DogLog.log("Ferry Pose", pose); // i will figure out how doglog works at some point
        return pose;
    }

   

	

	public static class DistanceA {
	// FOR THE LOVE OF ALL THAT IS HOLY DO NOT NAME THIS 'DISTANCE' THERE ARE ALREADY 3 WPILIB CLASSES NAMED THAT
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;

		public static final PIDController rotationController = getRotationController();

		private static final PIDController getRotationController() {
			PIDController controller = new PIDController(2.0, 0.0, 0.0);
			controller.enableContinuousInput(-Math.PI, Math.PI);
			return controller;
		}
	}

	public static class ComponentIDs {
		public static final int PigeonID = 20;
		
	}

	public static double GetmaximumSpeed() {
		return maximumSpeed;
	}

	public static double GetmaximumangularSpeed() {
		return maximumAngleSpeed;
	}
}
