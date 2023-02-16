package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.MotorUtil;
import frc.robot.util.ShuffleboardUtil;

import static frc.robot.Constants.TiltedElevatorConstants.*;

public class TiltedElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollow;
    private final CANSparkMax extensionFollowB;

    private final DigitalInput zeroLimitSwitch;

    private static final double EXTENSION_GEAR_RATIO = 14.0 / 64.0;
    private static final double EXTENSION_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
    private static final double EXTENSION_ROTATIONS_TO_METERS = EXTENSION_GEAR_RATIO * EXTENSION_CIRCUMFERENCE * 2.0 * (15.0 / 13.4);
    private static final float EXTENSION_LIMIT = 26.0f;

    private static final double extensionP = 0.25; //.4 // 4.9;
    private static final double extensionI = 0;
    private static final double extensionD = 0.9; //.2
    private static final double extensionFF = 0.3; //0.1
    private static final double maxVel = 1.1; // m/s
    private static final double maxAccel = 1.8; // 0.6 // m/s^2
    private static final double extensionTolerance = 0.003;
    private double arbFeedforward = 0.02;

    private ElevatorState state = ElevatorState.GROUND;

    private static final boolean IS_MANUAL = false;
    private double manualPower = 0;

    private static final double OFFSET_FACTOR = 0.01; // The factor to multiply driver input by when changing the offset.
    private double offsetDistMeters = 0;

    public boolean pieceGrabbed = false;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry 
        extensionPEntry, extensionIEntry, extensionDEntry, extensionFFEntry,
        maxVelEntry, maxAccelEntry, extensionToleranceEntry, arbFFEntry;
    private final GenericEntry manualPowerEntry, targetExtensionEntry;
    private final GenericEntry currentExtensionEntry, currentVelEntry, currentStateEntry, offsetDistEntry;

    public enum ElevatorState {
        GROUND(0) {
            @Override
            public double getExtension(boolean hasPiece) {
                // If a piece is being held, raise it slightly so that it doesn't drag across the floor.
                return hasPiece 
                    ? extendDistanceMeters + Units.inchesToMeters(5)
                    : extendDistanceMeters;
            }
        },
        CHUTE(Units.inchesToMeters(40)),
        SUBSTATION(Units.inchesToMeters(50)), // absolute height = 37.375 in
        CUBE_MID(Units.inchesToMeters(33)), // absolute height = 14.25 in
        CUBE_HIGH(Units.inchesToMeters(53)), // absolute height = 31.625 in
        CONE_MID(Units.inchesToMeters(50)), // absolute height = 34 in
        CONE_HIGH(Units.inchesToMeters(0)); // absolute height = 46 in

        protected double extendDistanceMeters; // meters, extension distance of winch

        /**
         * ElevatorState defined by extension of elevator from zero. All values in meters and radians.
         * @param extendDistanceMeters The extension of the subsystem.
         */
        private ElevatorState(double extendDistanceMeters) {
            this.extendDistanceMeters = extendDistanceMeters;
        }

        /**
         * Gets the extension commanded by the elevator state.
         * @param hasPiece Whether there is a game piece in the subsystem.
         * @return The distance, in meters, the elevator should extend to.
         */
        public double getExtension(boolean hasPiece) {
            return this.extendDistanceMeters;
        }
    }

    public TiltedElevatorSubsystem() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
            extensionEncoder.setVelocityConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60.0);
            extensionEncoder.setPosition(0);

            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, EXTENSION_LIMIT);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));

            extensionPidController = MotorUtil.createSparkMaxPIDController(sparkMax, extensionEncoder);
            extensionPidController.setP(extensionP);
            extensionPidController.setI(extensionI);
            extensionPidController.setD(extensionD);
            extensionPidController.setFF(extensionFF);
            extensionPidController.setSmartMotionAllowedClosedLoopError(extensionTolerance, 0);
            extensionPidController.setSmartMotionMaxVelocity(maxVel, 0);
            extensionPidController.setSmartMotionMaxAccel(maxAccel, 0);
            extensionPidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        });

        extensionFollow = MotorUtil.createSparkMax(EXTENSION_FOLLOW_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        if (Constants.IS_R1) extensionFollowB = null;
        else extensionFollowB = MotorUtil.createSparkMax(EXTENSION_FOLLOW_B_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        if (Constants.IS_R1) zeroLimitSwitch = null;
        else zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_ID);

        // TODO: positions
        shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");

        extensionPEntry = shuffleboardTab.add("Extension P", extensionP).withPosition(0, 0).getEntry();
        extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
        extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();
        extensionFFEntry = shuffleboardTab.add("Extension FF", extensionFF).getEntry();
        maxVelEntry = shuffleboardTab.add("Max vel", maxVel).getEntry();
        maxAccelEntry = shuffleboardTab.add("Max accel", maxAccel).getEntry();
        extensionToleranceEntry = shuffleboardTab.add("Extension tolerance", extensionTolerance).getEntry();
        arbFFEntry = shuffleboardTab.add("Arb FF", arbFeedforward).getEntry();

        manualPowerEntry = shuffleboardTab.add("Manual Power", manualPower).getEntry();
        targetExtensionEntry = shuffleboardTab.add("Target Ext (in)", 0.0).getEntry();

        currentStateEntry = shuffleboardTab.add("Current state", state.toString()).getEntry();
        currentExtensionEntry = shuffleboardTab.add("Current Ext (in)", 0.0).getEntry();
        currentVelEntry = shuffleboardTab.add("Current Vel (mps)", 0.0).getEntry();
        offsetDistEntry = shuffleboardTab.add("Offset (in)", offsetDistMeters).getEntry();
    }

    @Override
    public void periodic() {
        if (zeroLimitSwitch != null && zeroLimitSwitch.get())
            extensionEncoder.setPosition(0); 

        // If we're in manual power mode, use percent out power supplied by driver joystick.
        if (IS_MANUAL) {
            manualPowerEntry.setDouble(manualPower);
            extensionMotor.set(manualPower);
            return;
        }

        ShuffleboardUtil.pollShuffleboardDouble(extensionPEntry, extensionPidController::setP);
        ShuffleboardUtil.pollShuffleboardDouble(extensionIEntry, extensionPidController::setI);
        ShuffleboardUtil.pollShuffleboardDouble(extensionDEntry, extensionPidController::setD);
        ShuffleboardUtil.pollShuffleboardDouble(extensionFFEntry, extensionPidController::setFF);
        ShuffleboardUtil.pollShuffleboardDouble(maxVelEntry, (value) -> extensionPidController.setSmartMotionMaxVelocity(value, 0));
        ShuffleboardUtil.pollShuffleboardDouble(maxAccelEntry, (value) -> extensionPidController.setSmartMotionMaxAccel(value, 0));
        ShuffleboardUtil.pollShuffleboardDouble(extensionToleranceEntry, (value) -> extensionPidController.setSmartMotionAllowedClosedLoopError(value, 0));
        arbFeedforward = arbFFEntry.getDouble(arbFeedforward);

        // System.out.println(extensionEncoder.getPosition());

        double currentPos = extensionEncoder.getPosition();
        double currentVel = extensionEncoder.getVelocity();
        double targetExtension = state.getExtension(pieceGrabbed) + offsetDistMeters;

        // If we're trying to get to 0, set the motor to 0 power so the carriage drops with gravity
        // and hits the hard stop / limit switch.
        if (targetExtension <= 0 && currentPos < Units.inchesToMeters(1)) {
            extensionMotor.set(0);
        } else if (targetExtension <= 0 && currentPos < Units.inchesToMeters(5)) {
            extensionMotor.set(-0.075);
        } else {
            // Set PID reference
            extensionPidController.setReference(
                MathUtil.clamp(targetExtension, 0, EXTENSION_LIMIT),
                ControlType.kSmartMotion, 0,
                arbFeedforward, ArbFFUnits.kPercentOut
            );
        }

        currentExtensionEntry.setDouble(Units.metersToInches(currentPos));
        currentVelEntry.setDouble(currentVel);
        currentStateEntry.setString(state.toString());
        targetExtensionEntry.setDouble(targetExtension);
        offsetDistEntry.setDouble(Units.metersToInches(offsetDistMeters));
    }

    /**
     * Sets the state of the subsystem.
     * @param state The state to set it to.
     */
    public void setState(ElevatorState state) {
        this.state = state;
    }

    /**
     * Toggles the state of the subsystem between the two specified `ElevatorState`s, defaulting
     * to state 1. Also resets the distance offset.
     * 
     * @param state1 The first state.
     * @param state2 The second state.
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) {
        resetOffset();
        state = state == state1
            ? state2
            : state1;
    }

    /**
     * Changes the distance offset from given [-1.0, 1.0] driver powers.
     * @param power The power to change the offset by.
     */
    public void changeOffsetDistMeters(double power) {
        this.offsetDistMeters += OFFSET_FACTOR * power;
    }

    /**
     * Zeros the distance offset.
     */
    public void resetOffset() {
        this.offsetDistMeters = 0;
    }

    /**
     * Sets the manual power of this subsystem.
     * @param power The manual [-1.0, 1.0] driver-controlled power.
     */
    public void setManualPower(double power) {
        this.manualPower = power;
    }
}
