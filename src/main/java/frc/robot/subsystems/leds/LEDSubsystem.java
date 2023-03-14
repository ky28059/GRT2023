package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDConstants.*;

import java.util.List;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private final double APRIL_BLINK_DURATION = .2;
    private static final Color BLINK_COLOR = new Color(0, 0, 0);
    private boolean blinking = false;
    private boolean continuous = false; //toggle in manual mode
    public boolean manual = false; //when driver is directly controlling leds

    private static final double BRIGHTNESS_SCALE_FACTOR = 0.25;
    private static final double INPUT_DEADZONE = .1;

    private Color color = new Color(192, 8, 254);
    private final Color APRIL_COLOR = new Color(255, 0, 0);
    private final Color CUBE_COLOR = new Color(192, 8, 254);
    private final Color CONE_COLOR = new Color(255, 100, 0);
    private final Timer aprilTimer = new Timer();
    private final Timer aprilTimer2 = new Timer();
    public boolean pieceGrabbed = false;

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);
        blinkTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Start blink timer loop if we are holding a piece
        if (pieceGrabbed) {
            blinkTimer.start();
        } else {
            blinking = false;
            blinkTimer.stop();
            blinkTimer.reset();
        }

        // Toggle the blink boolean every duration to swap the LEDs between the driver piece color
        // and the blink color.
        if (blinkTimer.advanceIfElapsed(BLINK_DURATION_SECONDS)) blinking = !blinking;
        if(manual){
            //if manual and continuous set the bottom to color
            ledStrip.updateContinuousColor(color);
            ledStrip.setContinuousColor();
        //} else if(manual){
            //if manual and not continuous fill entire strip with color
            // ledStrip.fillContinuousColor(color);
            // ledStrip.setContinuousColor();
        } else {
            setColorPulse();
            //setTwoColor()
        }
    }

    /**
     * Sets the driver-provided color of the LEDs from provided RGB values.
     * @param r The red component of the color, from [0, 255].
     * @param g The green component of the color, from [0, 255].
     * @param b The blue component of the color, from [0, 255].
     */
    public void setRGB(double r, double g, double b) {
        //if the color doesnt change, dont do anything
        if(this.color.red == r && this.color.green == g && this.color.blue == b){ 
            return;
        }
        this.color = new Color(
            (int) (r * BRIGHTNESS_SCALE_FACTOR),
            (int) (g * BRIGHTNESS_SCALE_FACTOR),
            (int) (b * BRIGHTNESS_SCALE_FACTOR)
        );
        //sets the whole strip to the color
        ledStrip.fillContinuousColor(this.color);
        ledStrip.setContinuousColor();
    }

    /**
     * Sets the driver-provided color of the LEDs from provided HSV values.
     * @param h The hue component of the color, from [0, 180).
     * @param s The saturation component of the color, from [0, 255].
     * @param v The value component of the color, from [0, 255].
     */
    //sets the color to the 
    public void setHSV(double h, double s, double v) {
        this.color = Color.fromHSV(
            (int) h,
            (int) s,
            (int) (v * BRIGHTNESS_SCALE_FACTOR)
        );
    }

    public void toggleLEDControlMode(){
        continuous = !continuous;
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public void tagDetected(){
        if(aprilTimer.hasElapsed(APRIL_BLINK_DURATION * 2)){
            aprilTimer.reset();
            aprilTimer.start();
        }
        aprilTimer2.reset();
        aprilTimer2.start();
    }

    public void setColorPulse(){
        Color currentColor;
        //current color is the color that will be added at the bottom of the strip buffer
        if(!aprilTimer.hasElapsed(APRIL_BLINK_DURATION)){
            currentColor = APRIL_COLOR;
        } else {
            currentColor = color;
        }
        if(blinking){
            ledStrip.fillContinuousColorIgnoringOneColor(BLINK_COLOR, APRIL_COLOR);
        }
        //if the leds are on update the continuous color and then set the leds to that continuous buffer
        ledStrip.updateContinuousColor(currentColor);
        ledStrip.setContinuousColor();
    }

    public void setTwoColor(){
        Color color2;
        if(blinking){
            color = BLINK_COLOR;
        }
        if(!aprilTimer2.hasElapsed(.05)){
            color2 = APRIL_COLOR;
        } else {
            color2 =  color;
        }
        ledStrip.setTwoColors(color, color2);
    }

    public void setColorFromXandY(double x, double y){
        if(manual){
            double angleRads = MathUtil.inputModulus(Math.atan2(y, x), 0, 2 * Math.PI);
            this.color = Color.fromHSV(
            (int) (angleRads / (2 * Math.PI) * 180),
            (int) 255,
            (int) (255 * BRIGHTNESS_SCALE_FACTOR)
        );
        } else {
            if(x > INPUT_DEADZONE){
                color = CUBE_COLOR;
            } else if(x < -INPUT_DEADZONE){
                color = CONE_COLOR;
            }
        }
    }
}
