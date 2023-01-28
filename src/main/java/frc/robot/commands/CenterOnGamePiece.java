package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CenterOnGamePiece extends DriveArcadeWithPID {

    public CenterOnGamePiece(Drivetrain drivetrain, Vision vision) {
        super(drivetrain, () -> vision.getLimelightYaw(), 0, 0, drivetrain.getAnglePIDSettings());
    }
}
