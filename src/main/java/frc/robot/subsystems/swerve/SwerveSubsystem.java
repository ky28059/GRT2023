package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule topLeftModule;
    private final SwerveModule topRightModule;
    private final SwerveModule bottomLeftModule;
    private final SwerveModule bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final AHRS ahrs;

    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = Math.toRadians(60); // Max robot angular velocity, in rads/s

    // The `SwerveModuleState` setpoints for each module;
    // states are given in a tuple of [top left, top right, bottom left, bottom right].
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private boolean locked = false;

    public SwerveSubsystem() {
        // Initialize swerve modules
        topLeftModule = new SwerveModule(tlDrive, tlSteer, tlOffsetRads);
        topRightModule = new SwerveModule(trDrive, trSteer, trOffsetRads);
        bottomLeftModule = new SwerveModule(blDrive, blSteer, blOffsetRads);
        bottomRightModule = new SwerveModule(brDrive, brSteer, brOffsetRads);

        // Initialize system kinematics with top left, top right, bottom left, and bottom right swerve
        // module positions.
        kinematics = new SwerveDriveKinematics(
            tlPos, trPos, blPos, brPos
        );

        // Initialize NaxX and pose estimator
        ahrs = new AHRS(SPI.Port.kMXP);
        poseEstimator = new SwerveDrivePoseEstimator(
            getGyroHeading(), 
            new Pose2d(), 
            kinematics, 
            // State measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            // Odometry measurement standard deviations: [gyro]
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), 
            // Vision measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    /**
     * Sets the swerve module states of this subsystem from provided field-centric swerve drive powers.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA,
            getGyroHeading()
        );

        // Calculate swerve module states from desired chassis speeds
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    /**
     * Sets the swerve module states of this subsystem. Module states are assumed to be passed
     * in a tuple of [top left, top right, bottom left, bottom right].
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        this.states = states;
    }

    /**
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.update(
            gyroAngle,
            topLeftModule.getState(),
            topRightModule.getState(),
            bottomLeftModule.getState(),
            bottomRightModule.getState()
        );

        // Set swerve modules to their setpoints if unlocked, or their locked positions otherwise.
        if (locked) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        } else {
            // Desaturate speeds to ensure all velocities are under MAX_VEL after kinematics.
            SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);
    
            topLeftModule.setDesiredState(states[0]);
            topRightModule.setDesiredState(states[1]);
            bottomLeftModule.setDesiredState(states[2]);
            bottomRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * Toggles whether the subsystem is locked (whether control input has no effect and the wheels 
     * should lock themselves to prevent shoving).
     */
    public void toggleLocked() {
        locked = !locked;
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
     * @param position The position to reset the pose estimator to.
     */
    public void resetPosition(Pose2d position) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(position, gyroAngle);
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPosition() {
        resetPosition(new Pose2d());
    }

    /**
     * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive.
     * @return The robot heading as a Rotation2d.
     */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }
}
