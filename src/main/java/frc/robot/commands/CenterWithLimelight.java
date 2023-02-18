package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import frc.robot.services.VisionService;
import frc.robot.services.VisionService.LimelightPipeline;
import frc.robot.subsystems.Drivetrain;

public class CenterWithLimelight extends DriveArcadeWithPID {

    private final VisionService vision;
    private final LimelightPipeline pipeline;

    public CenterWithLimelight(Drivetrain drivetrain, VisionService vision, LimelightPipeline pipeline) {
        super(drivetrain, vision::getLimelightYaw, 0, 0, drivetrain.getCameraPIDSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        vision.setLimelightPipeline(pipeline);
    }
}
