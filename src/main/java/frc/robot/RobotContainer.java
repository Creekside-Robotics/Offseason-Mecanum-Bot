// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick joystick = new Joystick(0);

  private Drivetrain drivetrain = new Drivetrain();
  private ResetGyro resetCommand = new ResetGyro(drivetrain);
  private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.drivetrain.setDefaultCommand(new ManualDrive(this.drivetrain, this.joystick));
    var resetButton = new JoystickButton(joystick, 11);
    var flagButton = new JoystickButton(joystick, 1);
    resetButton.whenPressed(this.resetCommand);
    flagButton.whenPressed(() -> {
      piston.set(Value.kForward);
    });
    flagButton.whenReleased(() -> {
      piston.set(Value.kReverse);
    });


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
