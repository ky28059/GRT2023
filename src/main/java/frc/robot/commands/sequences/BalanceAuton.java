package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The middle auton sequence, where the robot deposits its preloaded game piece and balances
 * on the charging station.
 */
public class BalanceAuton extends BaseAutonSequence {
    /**
     * Constructs an auton sequence from the given parameters.
     * @param robotContainer The robot container.
     * @param initialPose The initial pose of the robot.
     * @param placePose The robot's pose when placing the preloaded game piece.
     * @param outsidePose The robot's pose outside of the community(for extra points).
     * @param height The height of the mover subsystem when placing the preloaded game piece.
     */
    public BalanceAuton(
        BaseSwerveSubsystem swerveSubsystem,
        RollerSubsystem rollerSubsystem,
        TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, 
        Pose2d placePose, 
        Pose2d outsidePose, 
        ElevatorState height
    ) {
        super(
            swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
            initialPose, placePose, outsidePose,
            height
        );
    }
}
