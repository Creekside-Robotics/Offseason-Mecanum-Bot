// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  CANSparkMax rightFront;
  CANSparkMax rightRear;
  CANSparkMax leftFront;
  CANSparkMax leftRear;

  MecanumDrive driveGroup;
  ADXRS450_Gyro gyro;

  public Drivetrain() {
    rightFront = new CANSparkMax(Constants.rightFrontID, CANSparkMax.MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.rightRearID, CANSparkMax.MotorType.kBrushless);
    leftFront = new CANSparkMax(Constants.leftFrontID, CANSparkMax.MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.leftRearID, CANSparkMax.MotorType.kBrushless);
    driveGroup = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
    gyro = new ADXRS450_Gyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Translation2d getFieldOrientedDriveVector(Translation2d driveVector) {
    var driveSpeed = driveVector.getNorm();
    var driveAngle = new Rotation2d(driveVector.getX(), driveVector.getY());
    var gyroAngle = gyro.getRotation2d();
    var fieldOrientedDriveAngle = driveAngle.minus(gyroAngle);

    return new Translation2d(fieldOrientedDriveAngle.getCos()*driveSpeed, fieldOrientedDriveAngle.getSin()*driveSpeed);
  }

  /**
   * Use this method to set the output drivetrain
   * @param joystickVector Translation2d vector {[-1, 1], [-1, 1]} setting the direction and speed in WPILIB field/robot relative corrdinate system https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html.
   * @param rotationalVelocity Rotational velocity of robot [-1, 1], counter-clockwise is positive.
   * @param fieldOriented Sets whether or not to use field oriented drive.
   */
  public void setDriveOutput(Translation2d joystickVector, double rotationalVelocity, boolean fieldOriented){
    Translation2d driveVector;
    if(fieldOriented){
      driveVector = this.getFieldOrientedDriveVector(joystickVector);
    } else {
      driveVector = joystickVector;
    }
    this.driveGroup.driveCartesian(driveVector.getX(), -driveVector.getY(), -rotationalVelocity);
  }
}
