package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class OI /*GEVALD*/ {

    private static OI instance;

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private final XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    private OI(Drivetrain drivetrain, ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper,
               ArmGravityCompensation compensation) {
        //Moves the first joint forward
        ps.getR1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        }.andThen(new KeepArmStable(firstJoint, secondJoint, compensation)));
        //Moves the first joint backwards
        ps.getR2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        }.andThen(new KeepArmStable(firstJoint, secondJoint, compensation)));
        //Moves the second joint forward
        ps.getL1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        }.andThen(new KeepArmStable(firstJoint, secondJoint, compensation)));
        //Moves the second joint backwards
        ps.getL2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        }.andThen(new KeepArmStable(firstJoint, secondJoint, compensation)));

        //Moves the arm to the floor
        ps.getCrossButton().onTrue(new MoveArmToFloor(firstJoint, secondJoint, compensation));
        //Places game piece in the middle
        ps.getTriangleButton().onTrue(new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_MID));
        //Switch sides of arm
//        ps.getSquareButton().onTrue(new SwitchSides(firstJoint, secondJoint, gripper));
//        ps.getSquareButton().onTrue(new InstantCommand(() -> {
//            if (secondJoint.isBack()) {
//                new SwitchSides(firstJoint, secondJoint, gripper, true).schedule();
//            } else {
//                new SwitchSides(firstJoint, secondJoint, gripper, false).schedule();
//
//            }
//        }));
        ps.getSquareButton().onTrue(new SwitchSides(firstJoint, secondJoint, gripper, true));
        ps.getCircleButton().onTrue(new SwitchSides(firstJoint, secondJoint, gripper, false));
        //Keeps the arm stable
        ps.getShareButton().whileTrue(new KeepArmStable(firstJoint, secondJoint, compensation));
        //Places game piece at the top
        ps.getOptionsButton().onTrue(new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP));
        //Stops both joints
        ps.getRightStickButton().onTrue(new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        //Opens the gripper
        ps.getUpButton().onTrue(new OpenGripper(gripper));
        //Closes the gripper
        ps.getDownButton().onTrue(new CloseGripper(gripper));

        xbox.getLeftStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
        xbox.getRightStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)));
        xbox.getButtonStart().onTrue(new Climb(drivetrain));

        xbox.getUpButton().onTrue(new InstantCommand(() -> {
        }, drivetrain));
        xbox.getLeftButton().onTrue(new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT));
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
