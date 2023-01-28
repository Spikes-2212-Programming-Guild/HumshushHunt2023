package frc.robot.subsystems;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Drivetrain extends TankDrivetrain {

    public Drivetrain(String namespaceName, MotorController left, MotorController right) {
        super(namespaceName, left, right);
    }

    public PIDSettings getAnglePIDSettings() {
        return null;
    }
}
