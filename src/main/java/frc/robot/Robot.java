// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.SetSparkMax;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class Robot extends TimedRobot {

//    Drivetrain drivetrain = Drivetrain.getInstance();
    PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    @Override
    public void robotInit() {
        ps.getCircleButton().whileTrue(new SetSparkMax(new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));
        ps.getCrossButton().whileTrue(new SetSparkMax(new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));
        ps.getSquareButton().whileTrue(new SetSparkMax(new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));
        ps.getTriangleButton().whileTrue(new SetSparkMax(new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));

        ps.getR1Button().whileTrue(new SetSparkMax(new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));
        ps.getR2Button().whileTrue(new SetSparkMax(new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));

        ps.getL1Button().whileTrue(new SetSparkMax(new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless), 0.2));
//
//        Gripper gripper = Gripper.getInstance();
//        ps.getUpButton().onTrue(new OpenGripper(gripper));
//        ps.getDownButton().onTrue(new CloseGripper(gripper));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
//        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, drivetrain::getRightY, drivetrain::getLeftX));
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
}
