package frc.robot.commands.athena;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.TurnInAngle;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

/**
 * The backend of student written programs.<br>
 * This class adds commands to a {@link SequentialCommandGroup} according to the user's demand.
 */
public abstract class ProgramBase extends SequentialCommandGroup {

    public ProgramBase() {
        writeProgram();
    }

    private final Drivetrain drivetrain = Drivetrain.getInstance();

    protected void moveForwardInSeconds(double seconds) {
        addCommands(new DriveArcade(drivetrain, Drivetrain.DRIVE_SPEED, 0).withTimeout(seconds));
    }

    protected void moveBackwardInSeconds(double seconds) {
        addCommands(new DriveArcade(drivetrain, -Drivetrain.DRIVE_SPEED, 0).withTimeout(seconds));
    }

    protected void turnLeftInDegrees(double angle) {
        addCommands(new TurnInAngle(drivetrain, -angle, true));
    }

    protected void turnRightInDegrees(double angle) {
        addCommands(new TurnInAngle(drivetrain, angle, false));
    }

    protected void releaseCube() {
        addCommands(new OpenGripper(Gripper.getInstance()));
    }

    protected void moveArm() {
        addCommands(new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                PlaceGamePiece.ArmState.FRONT_MID));
    }

    protected abstract void writeProgram();
}