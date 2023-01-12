package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

    double xPower;
    double yPower;
    double angularPower;

    public void setDrivePowers(double forwardComponent, double sideComponent, double angularPower){
        xPower = forwardComponent;
        yPower = sideComponent;
        this.angularPower = angularPower;
    }

}
