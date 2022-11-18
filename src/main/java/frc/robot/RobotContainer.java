// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shuffleboard.GRTShuffleboardTab;
import frc.robot.subsystems.AlignerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.TankSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    // private final SwerveSubsystem swerveSubsystem;
    final GripperSubsystem gripper = new GripperSubsystem();
    final ElevatorSubsystem elevator = new ElevatorSubsystem();
    final TankSubsystem tank = new TankSubsystem();
    final AlignerSubsystem aligner = new AlignerSubsystem();
    private final XboxController driver = new XboxController(0);
    private final XboxController stacker = new XboxController(1);

    // Controllers and buttons

    // Commands
    private final SendableChooser<Command> autonChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());

        new GRTShuffleboardTab("Drivetrain").addWidget("Auton sequence", autonChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public void periodic() {
        // Left joystick forward/reverse
        // Right joystick left/right
        // Tank drive scaled by a quadratic for more precise movement
        tank.forwardpower = driver.getLeftY() * Math.abs(driver.getLeftY());
        tank.turnpower = driver.getRightX() * Math.abs(driver.getRightX());

        // a button on the stacker controller toggles gripping
        if (stacker.getAButtonPressed() == true) {
            gripper.open = !gripper.open;
        }

        // bumpers move the winch up and down, they do wrap around
        if (stacker.getRightBumperPressed()) {
            elevator.height = elevator.height.next();
        }
        if (stacker.getLeftBumperPressed()) {
            elevator.height = elevator.height.previous();
        }

    }
}
