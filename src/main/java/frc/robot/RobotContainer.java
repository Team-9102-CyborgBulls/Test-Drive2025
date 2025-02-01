// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveCmd driveCmd = new DriveCmd(driveSubsystem);
  
  public static CommandXboxController manette = new CommandXboxController(0);

 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Trigger rBumper = manette.rightBumper();
    Trigger lBumper = manette.leftBumper();

    driveSubsystem.setDefaultCommand(
        driveSubsystem.arcadeDriveCommand(
            () -> manette.getLeftY(), () -> manette.getRightX()));

    manette
        .a()
        
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    manette
        .b()
        
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    manette
        .x()
        
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    manette
        .y()
        
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    rBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
    lBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
