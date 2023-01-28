package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CenterOnGamePiece extends DriveArcadeWithPID {

    private final Vision vision;
    private final int pipeline;

    public CenterOnGamePiece(Drivetrain drivetrain, Vision vision, int pipeline) {
        super(drivetrain, vision::getPhotonVisionYaw, 0, 0, drivetrain.getAnglePIDSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        vision.changePhotonVisionPipeline(pipeline);
    }
}
