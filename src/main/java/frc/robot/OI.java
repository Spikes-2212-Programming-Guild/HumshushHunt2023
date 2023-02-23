package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.VisionService;
import frc.robot.subsystems.*;

public class OI /*GEVALD*/ {

    private static OI instance;

    public final Command centerOnHighRRT;

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private final XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    private OI(Drivetrain drivetrain, ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper,
               ArmGravityCompensation compensation) {
        FakeArm fakeArm = FakeArm.getInstance();
        //Moves the first joint forward
        ps.getR1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void initialize() {
                super.initialize();
                addRequirements(fakeArm);
            }
        });
        //Moves the first joint backwards
        ps.getR2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void initialize() {
                super.initialize();
                addRequirements(fakeArm);
            }
        });
        //Moves the second joint forward
        ps.getL1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }

//            @Override
//            public void initialize() {
//                super.initialize();
//                addRequirements(fakeArm);
//            }
        });
        //Moves the second joint backwards
        ps.getL2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }

//            @Override
//            public void initialize() {
//                super.initialize();
//                addRequirements(fakeArm);
//            }
        });
        //Moves the arm to the floor
        ps.getCrossButton().onTrue(new ConditionalCommand(new MoveArmToFloor(firstJoint, secondJoint, compensation, true),
                new MoveArmToFloor(firstJoint, secondJoint, compensation, false), secondJoint::isBack));
        //Places game piece in the middle
        ps.getCircleButton().onTrue(new ConditionalCommand(new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_MID),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_MID), secondJoint::isBack));
        //Switch sides of arm
        ps.getSquareButton().onTrue(new ConditionalCommand(new SwitchSides(firstJoint, secondJoint, gripper, true),
                new SwitchSides(firstJoint, secondJoint, gripper, false), secondJoint::isBack));
        //Places game piece at the top
        ps.getTriangleButton().onTrue(new ConditionalCommand(
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_TOP),
                secondJoint::isBack));
        //lifts the arm to pick up from above
        ps.getLeftButton().onTrue(
//                new ConditionalCommand(
//                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_LIFT),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_LIFT));
//                secondJoint::isBack));
        //Keeps the arm stable
        ps.getShareButton().whileTrue(new KeepArmStable(firstJoint, secondJoint, compensation));
        //Stops both joints
        ps.getRightStickButton().onTrue(new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        //Opens the gripper
        ps.getUpButton().onTrue(new OpenGripper(gripper));
        //Closes the gripper
        ps.getDownButton().onTrue(new CloseGripper(gripper));
        //Folds the arm
        ps.getOptionsButton().onTrue(new ConditionalCommand(new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, () -> 0.1,
                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration), new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition, () -> 0.1,
                () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration), secondJoint::isBack));
        xbox.getLeftStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
        xbox.getRightStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)));
        xbox.getButtonStart().onTrue(new Climb(drivetrain));

        xbox.getUpButton().onTrue(new InstantCommand(() -> {
        }, drivetrain));
        centerOnHighRRT = new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT);
        xbox.getLeftButton().onTrue(centerOnHighRRT);
        xbox.getRightButton().onTrue(new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.LOW_RRT));
        xbox.getDownButton().onTrue(new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.APRIL_TAG));
        xbox.getLeftStickButton().onTrue(new InstantCommand(() -> {
        }, drivetrain));
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI(Drivetrain.getInstance(), ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                    Gripper.getInstance(), ArmGravityCompensation.getInstance());
        }
        return instance;
    }

    public double getRightY() {
        double val = xbox.getRightY();
        return Math.signum(val) * val * val;
    }

    public double getLeftX() {
        double val = xbox.getLeftX();
        return Math.signum(val) * val * val;
    }
}
