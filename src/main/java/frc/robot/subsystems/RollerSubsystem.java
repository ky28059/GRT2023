package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.util.MotorUtil;
import frc.robot.util.TrackingTimer;

import static frc.robot.Constants.RollerConstants.*;

public class RollerSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftBeak;
    private final WPI_TalonSRX rightBeak;
    private final WPI_TalonSRX openMotor;

    private final DigitalInput limitSwitch;
    private ColorSensorV3 crolorSensor;

    public enum HeldPiece {
        CONE, CUBE, EMPTY;
    }

    private HeldPiece heldPiece = HeldPiece.EMPTY;
    private HeldPiece lastHeldPiece = HeldPiece.EMPTY;

    public boolean allowOpen = true;
    private boolean prevAllowedOpen = true;
    private boolean colorSensorConnected = true;

    private final TrackingTimer openTimer = new TrackingTimer();
    private final TrackingTimer closeTimer = new TrackingTimer();
    private final TrackingTimer cooldownTimer = new TrackingTimer();

    private static final double OPEN_TIME_SECONDS = 1.0;
    private static final double CLOSE_TIME_SECONDS = 0.5;
    private static final double COOLDOWN_SECONDS = 2.0;

    // ~110 for nothing, 130 for cone, 160 for cube
    private static final int CONE_PROXIMITY_THRESHOLD = 115;
    private static final int CUBE_PROXIMITY_THRESHOLD = 145;

    private static final int CONE_RED = 92;
    private static final int CONE_GREEN = 140;
    private static final int CONE_BLUE = 22;

    private static final int CUBE_RED = 51;
    private static final int CUBE_GREEN = 78;
    private static final int CUBE_BLUE = 127;

    private static final int EMPTY_RED = 68;
    private static final int EMPTY_GREEN = 120;
    private static final int EMPTY_BLUE = 67;

    //for tuning
    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry limitEntry, proximityEntry, colorEntry, heldPieceEntry;
    private final GenericEntry rEntry, gEntry, bEntry, connectedEntry;

    private double rollPower = 0.0;

    public RollerSubsystem() {
        leftBeak = MotorUtil.createTalonSRX(LEFT_ID);
        // leftBeak.setInverted(true);
        leftBeak.setInverted(false);
        leftBeak.setNeutralMode(NeutralMode.Brake);

        rightBeak = MotorUtil.createTalonSRX(RIGHT_ID);
        rightBeak.follow(leftBeak);
        rightBeak.setInverted(InvertType.OpposeMaster);
        rightBeak.setNeutralMode(NeutralMode.Brake);

        openMotor = MotorUtil.createTalonSRX(OPEN_ID);
        openMotor.setNeutralMode(NeutralMode.Coast);
        // openMotor.setInverted(true);
        openMotor.setInverted(false);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
        crolorSensor = new ColorSensorV3(I2C.Port.kMXP);

        shuffleboardTab = Shuffleboard.getTab("Roller");
        limitEntry = shuffleboardTab.add("Limit piece", "EMPTY")
            .withPosition(0, 0)
            .getEntry();
        proximityEntry = shuffleboardTab.add("Proximity piece", "EMPTY")
            .withPosition(1, 0)
            .getEntry();
        colorEntry = shuffleboardTab.add("Color piece", "EMPTY")
            .withPosition(2, 0)
            .getEntry();
        heldPieceEntry = shuffleboardTab.add("Held piece", "EMPTY")
            .withPosition(3, 0)
            .getEntry();

        rEntry = shuffleboardTab.add("R", 0)
            .withPosition(0, 1)
            .getEntry();
        gEntry = shuffleboardTab.add("G", 0)
            .withPosition(1, 1)
            .getEntry();
        bEntry = shuffleboardTab.add("B", 0)
            .withPosition(2, 1)
            .getEntry();
        connectedEntry = shuffleboardTab.add("Sensor connected", 0)
            .withPosition(3, 1)
            .getEntry();
    }

    /**
     * Opens the roller by starting the open timer. Does nothing if opening is still on cooldown.
     */
    public void openMotor() {
        if (cooldownTimer.hasStarted() && !cooldownTimer.hasElapsed(COOLDOWN_SECONDS)) return;

        openTimer.start();
        closeTimer.stop();
        closeTimer.reset();
        cooldownTimer.stop();
        cooldownTimer.reset();
    }

    /**
     * Set the roller power of this subsystem.
     * @param power The power to set.
     */
    public void setRollPower(double power) {
        this.rollPower = power;
    }

    @Override
    public void periodic() {
        rollingLogic();
        openingLogic();
    }

    /**
     * Detects what game piece the intake is currently holding and applies power to
     * the roll motors.
     */
    private void rollingLogic() {
        HeldPiece limitPiece = getLimitSwitchPiece();
        // HeldPiece proximityPiece = getProximitySensorPiece();
        HeldPiece colorPiece = getColorSensorPiece();

        // If either the limit switch or the proximity sensor have detected a piece, set
        // the piece to the detected piece, prioritizing the proximity sensor over the limit
        // switch.
        if (colorPiece != HeldPiece.EMPTY) {
            heldPiece = colorPiece;
        } else {
            heldPiece = limitPiece;
        }

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (limitPiece == HeldPiece.EMPTY) {
            leftBeak.set(rollPower);
        } else {
            leftBeak.set(Math.min(rollPower, 0.0));
        }

        limitEntry.setString(limitPiece.name());
        // proximityEntry.setString(proximityPiece.name());
        colorEntry.setString(colorPiece.name());
        heldPieceEntry.setString(heldPiece.name());
    }

    /**
     * Applies power to the opening motor.
     */
    private void openingLogic() {
        boolean stopOpening = !allowOpen && prevAllowedOpen; // only heed falling edge of allowed open so we can open from ground position
        prevAllowedOpen = allowOpen;

        // If we're opening and not allowed to anymore, or if we've finished opening, stop opening and start closing.
        if ((openTimer.hasStarted() && stopOpening) || openTimer.hasElapsed(OPEN_TIME_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
            closeTimer.start();
            cooldownTimer.start();
        }

        // If we're closing and finished, stop closing.
        if (closeTimer.hasElapsed(CLOSE_TIME_SECONDS)) {
            closeTimer.stop();
            closeTimer.reset();
        }

        // Otherwise, open if we're opening and close if we're closing.
        if (openTimer.hasStarted()) openMotor.set(0.5);
        else if (closeTimer.hasStarted()) openMotor.set(-0.2);
        else if (heldPiece == HeldPiece.CONE) openMotor.setVoltage(-4.0);
        else if (rollPower > 0) openMotor.setVoltage(-1.0);
        else openMotor.set(0);
    }

    /**
     * Gets the piece detected by the limit switch.
     * @return The piece detected by the limit switch. This is either `EMPTY` if unpressed or `CONE` if pressed.
     */
    private HeldPiece getLimitSwitchPiece() {
        return !limitSwitch.get() ? HeldPiece.CUBE : HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the proximity measurement on the color sensor.
     * @return The piece detected by the proximity sensor.
     */
    private HeldPiece getProximitySensorPiece() {
        double dist = crolorSensor.getProximity();
        if (dist >= CUBE_PROXIMITY_THRESHOLD) return HeldPiece.CUBE;
        if (dist >= CONE_PROXIMITY_THRESHOLD) return HeldPiece.CONE;

        return HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the color sensor.
     * @return The piece detected by the color sensor.
     */
    private HeldPiece getColorSensorPiece() {
        // If we're not connected, return EMPTY and fall through to limit switch.
        colorSensorConnected = crolorSensor.isConnected();
        connectedEntry.setBoolean(colorSensorConnected);
        if (!colorSensorConnected) return HeldPiece.EMPTY;

        Color detectedColor = crolorSensor.getColor();
        double red = detectedColor.red * 255;
        double green = detectedColor.green * 255;
        double blue = detectedColor.blue * 255;

        rEntry.setValue(red);
        gEntry.setValue(green);
        bEntry.setValue(blue);

        // Calculate 3d distance between measured RGB and the EMPTY, CONE, and CUBE RGB values.
        double emptyDist = Math.pow(EMPTY_RED - red, 2) + Math.pow(EMPTY_GREEN - green, 2) + Math.pow(EMPTY_BLUE - blue, 2);
        double coneDist = Math.pow(CONE_RED - red, 2) + Math.pow(CONE_GREEN - green, 2) + Math.pow(CONE_BLUE - blue, 2);
        double cubeDist = Math.pow(CUBE_RED - red, 2) + Math.pow(CUBE_GREEN - green, 2) + Math.pow(CUBE_BLUE - blue, 2);

        // Return the piece with the least 3d distance.
        if (cubeDist < coneDist && cubeDist < emptyDist) {
            return HeldPiece.CUBE;
        } else if (coneDist < emptyDist) {
            return HeldPiece.CONE;
        } else {
            return HeldPiece.EMPTY;
        }
    }

    /**
     * Gets the currently held piece in the subsystem, or `HeldPiece.EMPTY` if there is no piece.
     * @return The held piece, or `HeldPiece.EMPTY`.
     */
    public HeldPiece getPiece() {
        return heldPiece;
    }

    public boolean colorSensorConnected() {
        return colorSensorConnected;
    }
}
