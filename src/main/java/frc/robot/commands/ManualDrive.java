// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Drive the robot with the joystick, through a  */
public class ManualDrive extends CommandBase {
  private Drivetrain drivetrain;
  private Joystick joystick;
  private boolean fieldOriented;

  /** Creates a new ManualDrive. */
  public ManualDrive(Drivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    this.fieldOriented = false;  // start as relative driving

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d joystickVector = new Translation2d(this.joystick.getX(), this.joystick.getY());
    double rotationalVelocity = this.joystick.getTwist();

    if (this.joystick.getRawButtonPressed(Constants.fieldOrientedDriveButton)) {  // only toggle when the button is pressed and not held
      this.fieldOriented = !this.fieldOriented;  // invert the toggle
    }

    this.drivetrain.setDriveOutput(
      joystickVector,
      rotationalVelocity,
      this.fieldOriented
    );

  }
}
