package frc.robot.subsystems;

import com.ctre.phoenix.platform.DeviceType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// MOST USEFUL IMPORTS ARE UNDER com.revrobotics.spark or edu.wpi.first.wpilibj

public class ElevatorSubsystemSimple extends SubsystemBase {
    private static SparkMax elevatorMotor1;
    private static SparkMax elevatorMotor2;


    public ElevatorSubsystemSimple() {
        elevatorMotor1 = new SparkMax(40, MotorType.kBrushed);
        elevatorMotor2 = new SparkMax(41, MotorType.kBrushed);
    }

    // Example intake function
    public static void up() {
        elevatorMotor1.set(1);
        elevatorMotor2.set(-1);
    }

    // Example score function
    public static void down() {
        elevatorMotor1.set(-1);
        elevatorMotor2.set(1);
    }

    public static void stop() {
        elevatorMotor1.stopMotor();
        elevatorMotor2.stopMotor();
    }

}
