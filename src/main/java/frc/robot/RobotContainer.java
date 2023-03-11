// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.auton.BalanceAndTaxiAutonSequence;
import frc.robot.commands.auton.BalanceAutonSequence;
import frc.robot.commands.auton.BottomBalanceAutonSequence;
import frc.robot.commands.auton.BottomOnePieceAutonSequence;
import frc.robot.commands.auton.BottomTwoPieceAutonSequence;
import frc.robot.commands.auton.PreloadedOnlyAutonSequence;
import frc.robot.commands.auton.TopOnePieceAutonSequence;
import frc.robot.commands.auton.TopTwoPieceAutonSequence;
import frc.robot.commands.auton.test.BoxAutonSequence;
import frc.robot.commands.auton.test.ContinuousBoxAutonSequence;
import frc.robot.commands.auton.test.GRTAutonSequence;
import frc.robot.commands.auton.test.HighRotationLinePath;
import frc.robot.commands.auton.test.RotatingSCurveAutonSequence;
import frc.robot.commands.auton.test.TenFeetStraightLinePath;
import frc.robot.commands.auton.test.TwentyFeetStraightLinePath;
import frc.robot.commands.balancing.BaseBalancerCommand;
import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.commands.dropping.AutoAlignGrid;
import frc.robot.commands.dropping.DropperChooserCommand;
import frc.robot.commands.pretest.MotorTestCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.TwistJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.vision.SwitchableCamera;
import frc.robot.vision.PhotonWrapper;
import frc.robot.subsystems.drivetrain.TankSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSweeperSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem2020;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.Superstructure;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final BaseDrivetrain driveSubsystem;
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final LEDSubsystem signalLEDSubsystem;

    private final Superstructure superstructure;
    private final PhotonWrapper photonWrapper;
    private final SwitchableCamera switchableCamera;

    // Controllers and buttons
    private final BaseDriveController driveController;

    private final GenericHID switchboard = new GenericHID(3);
    private final JoystickButton
        tlSwitch = new JoystickButton(switchboard, 3),
        tmSwitch = new JoystickButton(switchboard, 2),
        trSwitch = new JoystickButton(switchboard, 1),
        mlSwitch = new JoystickButton(switchboard, 6),
        mmSwitch = new JoystickButton(switchboard, 5),
        mrSwitch = new JoystickButton(switchboard, 4),
        blSwitch = new JoystickButton(switchboard, 9),
        bmSwitch = new JoystickButton(switchboard, 8),
        brSwitch = new JoystickButton(switchboard, 7);

    private final XboxController mechController = new XboxController(2);
    private final JoystickButton
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value),
        mechLBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value),
        mechRBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);

    // Commands
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
    private final SendableChooser<Command> autonChooser;
    private final MotorTestCommand testCommand;

    private final BaseBalancerCommand balancerCommand;
    private final AutoAlignGrid autoAlignCommands;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new DualJoystickDriveController();

        photonWrapper = new PhotonWrapper();
        switchableCamera = new SwitchableCamera(shuffleboardTab);

        // driveSubsystem = new MissileShellSwerveSubsystem();
        driveSubsystem = new SwerveSubsystem(photonWrapper);
        rollerSubsystem = new RollerSubsystem();
        tiltedElevatorSubsystem = new TiltedElevatorSubsystem();
        signalLEDSubsystem = new LEDSubsystem(); 

        superstructure = new Superstructure(rollerSubsystem, tiltedElevatorSubsystem, signalLEDSubsystem, switchableCamera);

        balancerCommand = new DefaultBalancerCommand(driveSubsystem);

        // Configure button bindings
        configureDriveBindings();
        configureMechBindings();

        // Initialize auton and test commands
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            testCommand = new MotorTestCommand(swerveSubsystem, tiltedElevatorSubsystem, rollerSubsystem);
            autoAlignCommands = new AutoAlignGrid(swerveSubsystem, tiltedElevatorSubsystem, false);

            autonChooser.addOption("Red preloaded only", new PreloadedOnlyAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            autonChooser.addOption("Red top auton (1-piece)", new TopOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            // autonChooser.addOption("Red top auton (2-piece)", new TopTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            autonChooser.addOption("Red balance auton", BalanceAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            // autonChooser.addOption("Red balance and taxi auton", BalanceAndTaxiAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            autonChooser.addOption("Red bottom auton (1-piece)", new BottomOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            // autonChooser.addOption("Red bottom auton (2-piece)", new BottomTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));
            // autonChooser.addOption("Red bottom balance auton", BottomBalanceAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, true));

            autonChooser.addOption("Blue preloaded only", new PreloadedOnlyAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            autonChooser.addOption("Blue top auton (1-piece)", new TopOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            // autonChooser.addOption("Blue top auton (2-piece)", new TopTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            autonChooser.addOption("Blue balance auton", BalanceAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            // autonChooser.addOption("Blue balance and taxi auton", BalanceAndTaxiAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            autonChooser.addOption("Blue bottom auton (1-piece)", new BottomOnePieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            // autonChooser.addOption("Blue bottom auton (2-piece)", new BottomTwoPieceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));
            // autonChooser.addOption("Blue bottom balance auton", BottomBalanceAutonSequence.withDeadline(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, false));

            // autonChooser.addOption("10' straight-line path", new TenFeetStraightLinePath(swerveSubsystem));
            // autonChooser.addOption("20' straight-line path", new TwentyFeetStraightLinePath(swerveSubsystem));
            // autonChooser.addOption("High rotation straight-line path", new HighRotationLinePath(swerveSubsystem));
            // autonChooser.addOption("Rotating S-curve", new RotatingSCurveAutonSequence(swerveSubsystem));
            // autonChooser.addOption("Box auton", new BoxAutonSequence(swerveSubsystem));
            // autonChooser.addOption("No-stopping box auton", new ContinuousBoxAutonSequence(swerveSubsystem));
            // autonChooser.addOption("GRT path", new GRTAutonSequence(swerveSubsystem));
        } else {
            testCommand = null;
            autoAlignCommands = null;
        }

        shuffleboardTab.add("Auton", autonChooser)
            .withPosition(8, 0)
            .withSize(4, 2);
    }

    /**
     * Configures button bindings for the drive subsystem and controller.
     */
    private void configureDriveBindings() {
        driveController.getBalancerButton().whileTrue(balancerCommand);
        driveController.getCameraSwitchButton().onTrue(new InstantCommand(switchableCamera::switchCamera));

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = driveController.getForwardPower();
                double yPower = driveController.getLeftPower();
                double angularPower = driveController.getRotatePower();
                boolean relative = driveController.getSwerveRelative();

                if (driveController.getSwerveHeadingLock()) {
                    double currentHeadingRads = swerveSubsystem.getFieldHeading().getRadians();
                    double lockHeadingRads = (Math.abs(currentHeadingRads) > Math.PI / 2.0) ? Math.PI : 0;

                    swerveSubsystem.setDrivePowersWithHeadingLock(xPower, yPower, new Rotation2d(lockHeadingRads), relative);
                } else {
                    swerveSubsystem.setDrivePowers(xPower, yPower, angularPower, relative);
                }
            }, swerveSubsystem));

            driveController.getFieldResetButton().onTrue(new InstantCommand(swerveSubsystem::resetFieldAngle, swerveSubsystem));
            driveController.getChargingStationLockButton().onTrue(new InstantCommand(swerveSubsystem::toggleChargingStationLocked, swerveSubsystem));

            driveController.getAlignToClosestButton().onTrue(autoAlignCommands.getAlignToClosestCommand());
            driveController.getCancelAutoAlignButton().onTrue(new InstantCommand(autoAlignCommands::cancelAll));

            /*
            brSwitch.onTrue(new InstantCommand(() -> {
                swerveSubsystem.setSteerRelativeEncoderFeedback(true);
            }, swerveSubsystem)).onFalse(new InstantCommand(() -> {
                swerveSubsystem.setSteerRelativeEncoderFeedback(false);
            }, swerveSubsystem));
            */
        } else if (driveSubsystem instanceof TankSubsystem) {
            final TankSubsystem tankSubsystem = (TankSubsystem) driveSubsystem;

            tankSubsystem.setDefaultCommand(new RunCommand(() -> {
                double forwardPower = 0.75 * driveController.getForwardPower();
                double turnPower = 0.75 * driveController.getRotatePower();
                tankSubsystem.setDrivePowers(forwardPower, turnPower);
            }, tankSubsystem));
        } else if (driveSubsystem instanceof MissileShellSwerveSubsystem) {
            final MissileShellSwerveSubsystem swerveSubsystem = (MissileShellSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = driveController.getForwardPower();
                double yPower = driveController.getLeftPower();
                swerveSubsystem.setDrivePowers(xPower, yPower);
            }, swerveSubsystem));
        }
    }

    /**
     * Configures button bindings for the roller / elevator mechs and controller.
     */
    private void configureMechBindings() {
        mechAButton.onTrue(new DropperChooserCommand(driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));

        rollerSubsystem.setDefaultCommand(new RunCommand(() -> {
            double forwardPower = 0.95 * mechController.getRightTriggerAxis();
            double reversePower = 0.65 * mechController.getLeftTriggerAxis();
            rollerSubsystem.setRollPower(forwardPower - reversePower);
        }, rollerSubsystem));

        mechYButton.onTrue(new InstantCommand(rollerSubsystem::openMotor, rollerSubsystem));

        tiltedElevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
            double yPower = -mechController.getLeftY();

            int pov = mechController.getPOV();
            switch (pov) {
                case 0 -> tiltedElevatorSubsystem.setState(ElevatorState.SUBSTATION);
                case 270 -> tiltedElevatorSubsystem.setState(ElevatorState.HYBRID);
                case 180 -> {
                    tiltedElevatorSubsystem.setState(ElevatorState.GROUND);
                    tiltedElevatorSubsystem.offsetState = OffsetState.OVERRIDE_HAS_PIECE;
                }
                case 90 -> tiltedElevatorSubsystem.resetOffset();
            }

            tiltedElevatorSubsystem.setManualPower(yPower);
            tiltedElevatorSubsystem.changeOffsetDistMeters(yPower);
        }, tiltedElevatorSubsystem));

        mechBButton.onTrue(new InstantCommand(() -> {
            // tiltedElevatorSubsystem.toggleState(ElevatorState.GROUND, ElevatorState.CHUTE);
            tiltedElevatorSubsystem.setState(ElevatorState.GROUND);
        }, tiltedElevatorSubsystem));

        mechXButton.onTrue(new InstantCommand(() -> {
            tiltedElevatorSubsystem.setState(ElevatorState.HOME);
        }, tiltedElevatorSubsystem));

        mechRBumper.onTrue(new InstantCommand(() -> {
            tiltedElevatorSubsystem.toggleState(ElevatorState.CUBE_MID, ElevatorState.CUBE_HIGH);
        }, tiltedElevatorSubsystem));

        mechLBumper.onTrue(new InstantCommand(() -> {
            tiltedElevatorSubsystem.toggleState(ElevatorState.CONE_MID, ElevatorState.CONE_HIGH);
        }, tiltedElevatorSubsystem));

        signalLEDSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (blSwitch.getAsBoolean()) {
                signalLEDSubsystem.setColor(255, 100, 0);
            } else {
                signalLEDSubsystem.setColor(192, 8, 254);
            }
        }, signalLEDSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /**
     * Use this to pass the test command to the main {@link Robot} class.
     * @return the command to run in test
     */
    public Command getTestCommand() {
        return testCommand;
    }
}
