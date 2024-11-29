package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class DropperChooserCommand extends ProxyCommand {
    public DropperChooserCommand(
        BaseDrivetrain driveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(() -> getSequence(driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
    }

    /**
     * Gets the drop sequence corresponding to the current ElevatorState.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    public static SequentialCommandGroup getSequence(
        BaseDrivetrain driveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        ElevatorState state = tiltedElevatorSubsystem.getState();
        return getSequence(driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, state);
    }

    /**
     * Gets the drop sequence corresponding to a given target ElevatorState.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @param state The target ElevatorState.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    public static SequentialCommandGroup getSequence(
        BaseDrivetrain driveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        ElevatorState state
    ) {
        return switch (state) {
            case CONE_HIGH -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0.2, 0.5
            );

            case CONE_MID -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0.2, 0.5
            );

            case CUBE_HIGH -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0.2, 0.5, 0.2, 0.5
            );

            case CUBE_MID -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0.2, 0.5, 0.2, 0.5
            );

            case GROUND -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0.65, 0.5, 0.2, 0.5
            );

            default -> new DropSequence(
                driveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0, 0.5
            );
        };
    }
}
