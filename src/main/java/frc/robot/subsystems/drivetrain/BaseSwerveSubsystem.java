package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * The superclass for the current `SwerveSubsystem` and `SwerveSubsystem2020` that contains all the
 * logic for managing module states, updating odometry, and taking driver input.
 */
public abstract class BaseSwerveSubsystem extends BaseDrivetrain {
    private final BaseSwerveModule topLeftModule;
    private final BaseSwerveModule topRightModule;
    private final BaseSwerveModule bottomLeftModule;
    private final BaseSwerveModule bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    public final double MAX_VEL; // Max robot tangential velocity, in m/s
    public final double MAX_ACCEL; // Max robot tangential acceleration, in m/s^2
    public final double MAX_OMEGA; // Max robot angular velocity, in rads/s

    private final Timer lockTimer;
    private static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking
    private static final boolean LOCKING_ENABLE = true;

    private Rotation2d angleOffset = new Rotation2d(0);

    // The driver or auton commanded `SwerveModuleState` setpoints for each module;
    // states are given in a tuple of [top left, top right, bottom left, bottom right].
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public BaseSwerveSubsystem(
        BaseSwerveModule topLeftModule,
        BaseSwerveModule topRightModule,
        BaseSwerveModule bottomLeftModule,
        BaseSwerveModule bottomRightModule,
        double maxVel, double maxAccel, double maxOmega,
        SwerveDriveKinematics kinematics
    ) {
        MAX_VEL = maxVel;
        MAX_ACCEL = maxAccel;
        MAX_OMEGA = maxOmega;

        this.topLeftModule = topLeftModule;
        this.topRightModule = topRightModule;
        this.bottomLeftModule = bottomLeftModule;
        this.bottomRightModule = bottomRightModule;

        this.kinematics = kinematics;

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroHeading(),
            getModuleStates(),
            new Pose2d(),
            // State measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            // Vision measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );

        lockTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.update(
            gyroAngle,
            getModuleStates()
        );

        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve modules if the lock timeout has elapsed, or set them to their
        // setpoints if drivers are supplying non-idle input.
        if (LOCKING_ENABLE && lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        } else {
            topLeftModule.setDesiredState(states[0]);
            topRightModule.setDesiredState(states[1]);
            bottomLeftModule.setDesiredState(states[2]);
            bottomRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * Sets the swerve module states of this subsystem from provided field-centric
     * swerve drive powers.
     * 
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     * @param relative Whether to use relative powers instead of field-oriented control.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower, boolean relative) {
        // If drivers are sending no input, stop all modules but hold their current angle.
        if (xPower == 0.0 && yPower == 0.0 && angularPower == 0.0) {
            this.states[0] = new SwerveModuleState(0.0, this.states[0].angle);
            this.states[1] = new SwerveModuleState(0.0, this.states[1].angle);
            this.states[2] = new SwerveModuleState(0.0, this.states[2].angle);
            this.states[3] = new SwerveModuleState(0.0, this.states[3].angle);
            return;
        }

        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL,
            yPower * MAX_VEL,
            angularPower * MAX_OMEGA,
            relative ? new Rotation2d() : getFieldHeading()
        );

        // Calculate swerve module states from desired chassis speeds, desaturating
        // them to ensure all velocities are under MAX_VEL after kinematics.
        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds, 
            MAX_VEL, MAX_VEL, MAX_OMEGA
        );
    }

    /**
     * Sets the swerve module states of this subsystem from provided relative drive powers.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction, relative to the robot.
     */
    @Override
    public void setDrivePowers(double xPower) {
        setDrivePowers(xPower, 0.0, 0.0, true);
    }

    /**
     * Sets the swerve module states of this subsystem. Module states are assumed to
     * be passed in a tuple of [top left, top right, bottom left, bottom right].
     * 
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        this.states = states;
    }

    /**
     * Gets the states of each module as a `SwerveModulePosition[]`.
     * @return The states of each module.
     */
    private SwerveModulePosition[] getModuleStates() {
        return new SwerveModulePosition[] {
            topLeftModule.getState(),
            topRightModule.getState(),
            bottomLeftModule.getState(),
            bottomRightModule.getState()
        };
    }

    /**
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot's position to a given Pose2d.
     * @param currentPose The position to reset the pose estimator to.
     */
    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle,
            getModuleStates(),
            currentPose
        );

        angleOffset = getGyroHeading().minus(currentPose.getRotation());
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPose() {
        resetPose(new Pose2d());
    }

    /**
     * Zeros *only the angle* of the robot's field-relative control system.
     * This also resets localization to match the rotated field.
     */
    public void resetFieldAngle(Rotation2d currentRotation) {
        Pose2d currPos = getRobotPosition();

        // Reset localization angle but keep current (x, y) to preserve the origin.
        resetPose(new Pose2d(
            currPos.getTranslation(),
            currentRotation
        ));
    }

    public void resetFieldAngle() {
        resetFieldAngle(new Rotation2d(0));
    }

    /**
     * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive.
     * @return The robot's global heading as a Rotation2d.
     */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    /**
     * Gets the angle of the robot relative to the field-relative control system.
     * This is equivalent to the robot's global angle with an offset applied.
     * 
     * @return The robot's field-centric heading as a Rotation2d.
     */
    private Rotation2d getFieldHeading() {
        // Primarily use AHRS reading, falling back on the pose estimator if the AHRS disconnects.
        return ahrs.isConnected()
            ? getGyroHeading().minus(angleOffset)
            : getRobotPosition().getRotation();
    }
}
