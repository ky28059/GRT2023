// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal Falcon motors

import edu.wpi.first.wpilibj.XboxController; // controller input 

import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax front = new CANSparkMax(FRONT_MOTOR, MotorType.kBrushless); // front motor for intake
    private final WPI_TalonSRX right = new WPI_TalonSRX​(RIGHT_MOTOR); // right roller between intake and carriage
    private final WPI_TalonSRX left = new WPI_TalonSRX​(LEFT_MOTOR); // left roller between intake and carriage

    Servo intake_servo = new Servo(INTAKE_SERVO);

    boolean intake_down = false;
    boolean intake_on = false;

  public Intake() {

    front.restoreFactoryDefaults();
    right.configFactoryDefault();
    left.configFactoryDefault();

    front.setIdleMode(kBrake);
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);

    left.follow(right);
    left.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(controller.getBButton()){ //engage the intake arm by pressing x on the controller
        intake_down = true;
        intake_servo.set(INTAKE_SERVO_DOWN);
    }
    if(controller.getXButton()){
        intake_on = !intake_on; //toggle on/off intake
    }

    if(intake_on){
        front.set(1.0);
        right.set(0.75);   
    }
    else{
        front.set(0.0);
        right.set(0.0);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}