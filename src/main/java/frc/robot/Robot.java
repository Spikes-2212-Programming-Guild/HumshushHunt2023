// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import com.spikes2212.command.drivetrains.commands.smartmotorcontrollerdrivetrain.MoveSmartMotorControllerTankDrivetrain;
import com.spikes2212.dashboard.AutoChooser;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.PlanBEdge;
import frc.robot.commands.autonomous.PlanBWindow;
import frc.robot.commands.autonomous.SmashAndDash;
import frc.robot.commands.autonomous.SplooshAndVamoose;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.LedsService;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class Robot extends TimedRobot {

    public static final RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;
    private ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
    private ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
    private Gripper gripper;
    private OI oi;
    private ArmGravityCompensation compensation;
    private VisionService vision;
    private LedsService leds;
    private AutoChooser autoChooser;

    @Override
    public void robotInit() {
        getInstances();
        setCompressor();
        setDefaultJointsCommands();
        setNamespaceTestingCommands();
        autoChooser = new AutoChooser(
                new RootNamespace("auto chooser"),
                new PlanBWindow(drivetrain).getCommand(), "plan b window",
                new PlanBEdge(drivetrain).getCommand(), "plan b edge",
                new SplooshAndVamoose(drivetrain).getCommand(), "sploosh and vamoose"
        );
        firstJoint.configureEncoders();
        secondJoint.configureEncoders();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        drivetrain.periodic();
        firstJoint.periodic();
        secondJoint.periodic();
        gripper.periodic();
        vision.periodic();
        leds.periodic();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        firstJoint.finish();
        secondJoint.finish();
        drivetrain.finish();
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }).ignoringDisable(true).schedule();
        new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)).ignoringDisable(true).schedule();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        });
//        new SplooshAndVamoose(drivetrain).getCommand().schedule();
//        autoChooser.schedule();
//        new SplooshAndVamoose(drivetrain).getCommand().schedule();
        new PlanBWindow(drivetrain).getCommand().schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }, firstJoint, secondJoint).schedule();
        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, oi::getRightY, oi::getLeftX));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    private void getInstances() {
        oi = OI.getInstance();
        drivetrain = Drivetrain.getInstance();
        compensation = ArmGravityCompensation.getInstance();
        firstJoint = ArmFirstJoint.getInstance();
        secondJoint = ArmSecondJoint.getInstance();
        gripper = Gripper.getInstance();
        vision = VisionService.getInstance();
        leds = LedsService.getInstance();
    }

    private void setCompressor() {
        Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        namespace.putData("enable compressor", new InstantCommand(compressor::enableDigital));
        namespace.putData("disable compressor", new InstantCommand(compressor::disable));
    }

    private void setDefaultJointsCommands() {
        firstJoint.setDefaultCommand(new KeepFirstJointStable(firstJoint, secondJoint, compensation));
        secondJoint.setDefaultCommand(new KeepSecondJointAngle(firstJoint, secondJoint, compensation));

//        FakeArm.getInstance().setDefaultCommand(new KeepArmStable(firstJoint, secondJoint, compensation));

//        firstJoint.setDefaultCommand(new InstantCommand(() -> compensation.configureFirstJointG(firstJointAngle, secondJointAngle)).
//                andThen(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.keepStablePIDSettings,
//                firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> firstJointAngle) {
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        }.alongWith(new RunCommand(() -> compensation.configureFirstJointG(firstJointAngle, secondJointAngle)))));
//
//        secondJoint.setDefaultCommand(new InstantCommand(() -> compensation.configureFirstJointG(firstJointAngle, secondJointAngle)).andThen(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.keepStablePIDSettings,
//                secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> secondJointAngle) {
//            @Override
//            public boolean isFinished() {
//                return false;
//            }
//        }.alongWith(new RunCommand(() -> compensation.configureSecondJointG(firstJointAngle, secondJointAngle)))));
    }

    private void setNamespaceTestingCommands() {
        namespace.putData("coast arm", new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }).ignoringDisable(true));
        namespace.putData("keep arm stable", new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()));
        namespace.putData("stop", new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        namespace.putData("move arm", new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP));

        namespace.putData("floor back", new MoveArmToFloor(firstJoint, secondJoint, compensation, true));
        namespace.putData("floor front", new MoveArmToFloor(firstJoint, secondJoint, compensation, false));
        namespace.putData("switch sides back", new SwitchSides(firstJoint, secondJoint, gripper, true));
        namespace.putData("switch sides front", new SwitchSides(firstJoint, secondJoint, gripper, false));
        namespace.putData("limelight center", new CenterWithFrontLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT));
        namespace.putData("climb", new Climb(drivetrain));
        namespace.putData("velocity drivetrain", new MoveSmartMotorControllerTankDrivetrain(drivetrain, drivetrain.getLeftPIDSettings(),
                drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings(),
                UnifiedControlMode.VELOCITY, () -> 3.0, () -> 3.0));
        namespace.putData("lift", new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_LIFT));
        namespace.putData("turn", new DriveArcadeWithPID(drivetrain, () -> -drivetrain.getYaw(), () -> 0.0, () -> 0.0,
                drivetrain.getCameraPIDSettings(), drivetrain.getFeedForwardSettings()) {
            @Override
            public void initialize() {
                feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(((Drivetrain) drivetrain).getYaw()));
            }

            @Override
            public void execute() {
                pidController.setTolerance(pidSettings.getTolerance());
                pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());

                feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                        feedForwardSettings.getkA(), feedForwardSettings.getkG());
                double calculate = pidController.calculate(source.get(), setpoint.get()) +
                        feedForwardController.calculate(setpoint.get());
                namespace.putNumber("turn calculate", calculate);
                drivetrain.arcadeDrive(moveValue.get(), calculate);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                feedForwardSettings.setkG(() -> 0.0);
            }
        });
        namespace.putData("smash and dash", new SmashAndDash(drivetrain).getCommand());
        namespace.putData("test test test", new ParallelCommandGroup(
                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                        () -> 0.005, () -> 0.7),
                new MoveFirstJoint(firstJoint, () -> 5.0, () -> 0.005, () -> 0.7)));
        namespace.putData("test test test 1", new ParallelCommandGroup(
                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.firstJointPosition,
                        () -> 0.005, () -> 0.7),
                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.secondJointPosition,
                        () -> 0.005, () -> 0.7)));
        namespace.putData("center on gp", new CenterOnGamePiece(drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE));
        namespace.putNumber("battery voltage", RobotController::getBatteryVoltage);
        namespace.putData("do path", new PPRamseteCommand(
                PathPlanner.loadPath("Smash And Dash", new PathConstraints(2.5, 2)),
                drivetrain::getPose2d, drivetrain.getRamseteController(),
                new SimpleMotorFeedforward(drivetrain.getFeedForwardSettings().getkS(),
                        drivetrain.getFeedForwardSettings().getkV(),
                        drivetrain.getFeedForwardSettings().getkA()),
                drivetrain.getKinematics(), () -> new DifferentialDriveWheelSpeeds(drivetrain.getLeftSpeed(), drivetrain.getRightSpeed()),
                new PIDController(drivetrain.getLeftPIDSettings().getkP(), drivetrain.getLeftPIDSettings().getkI(), drivetrain.getLeftPIDSettings().getkD()),
                new PIDController(drivetrain.getRightPIDSettings().getkP(), drivetrain.getRightPIDSettings().getkI(), drivetrain.getRightPIDSettings().getkD()),
                drivetrain::tankDriveVoltages, drivetrain));
        namespace.putData("sploosh and vamoose", new SplooshAndVamoose(drivetrain).getCommand());
        namespace.putData("climb2", new Climb2(drivetrain));
        namespace.putData("turn to 0", new TurnToZero(drivetrain));
    }
}
