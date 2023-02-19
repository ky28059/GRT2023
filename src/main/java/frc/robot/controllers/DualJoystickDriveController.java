package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A two-joystick drive controller on ports 0 and 1. Swerve relative is bound to the right trigger,
 * balancing is bound to the left trigger, and field reset is bound to the left middle button.
 */
public class DualJoystickDriveController extends BaseDriveController {
    private final Joystick leftJoystick = new Joystick(0);
    private final JoystickButton
        leftTrigger = new JoystickButton(leftJoystick, 1),
        leftMiddleButton = new JoystickButton(leftJoystick, 2),
        leftTopLeftButton = new JoystickButton(leftJoystick, 3),
        leftTopRightButton = new JoystickButton(leftJoystick, 4),
        leftMiddleLeftButton = new JoystickButton(leftJoystick, 5),
        leftMiddleRightButton = new JoystickButton(leftJoystick, 6),
        leftBackButton = new JoystickButton(leftJoystick, 7);

    private final Joystick rightJoystick = new Joystick(1);
    private final JoystickButton
        rightTrigger = new JoystickButton(rightJoystick, 1),
        rightStickBackButton = new JoystickButton(rightJoystick, 2),
        rightStickCenterButton = new JoystickButton(rightJoystick, 3),
        rightStickLeftButton = new JoystickButton(rightJoystick, 4),
        rightStickRightButton = new JoystickButton(rightJoystick, 5),
        rightBaseLeftTopButton = new JoystickButton(rightJoystick, 6),
        rightBaseLeftBottomButton = new JoystickButton(rightJoystick, 7),
        rightBaseBackLeftButton = new JoystickButton(rightJoystick, 8),
        rightBaseBackRightButton = new JoystickButton(rightJoystick, 9),
        rightBaseRightBottomButton = new JoystickButton(rightJoystick, 10),
        rightBaseRightTopButton = new JoystickButton(rightJoystick, 11);

    private final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForwardPower() {
        return MathUtil.applyDeadband(-leftJoystick.getY(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeftPower() {
        return MathUtil.applyDeadband(-leftJoystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotatePower() {
        return MathUtil.applyDeadband(-rightJoystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public boolean getSwerveRelative() {
        return rightJoystick.getTrigger();
    }

    @Override
    public JoystickButton getBalancerButton() {
        return leftTrigger;
    }

    @Override
    public JoystickButton getflipConeButton(){
        return leftMiddleLeftButton;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return leftMiddleButton;
    }

    @Override
    public JoystickButton getCameraSwitchButton() {
        return leftTopRightButton;
    }

    @Override
    public JoystickButton getChargingStationLockButton() {
        return leftTopLeftButton;
    }
}
