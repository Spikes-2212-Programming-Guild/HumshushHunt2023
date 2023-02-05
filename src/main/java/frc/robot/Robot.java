// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DrivoTankWithPID;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    Drivetrain drivetrain = Drivetrain.getInstance();
    DriveArcade command = new DriveArcade(drivetrain, 0.2, 0) {
        @Override
        public void execute() {
//                super.execute();
            drivetrain.setSpeeds(moveValueSupplier.get(), moveValueSupplier.get());
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setSpeeds(0, 0);
//            drivetrain.stop();
        }
    };
    PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    private RootNamespace name = new RootNamespace("jejoha");
    private Supplier<Double> set1 = name.addConstantDouble("s1", 8);
    private Supplier<Double> set2 = name.addConstantDouble("s2", 8);
    private Supplier<Double> src1 = name.addConstantDouble("sr1", 0);
    private Supplier<Double> src2 = name.addConstantDouble("sr2", 0);

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        drivetrain.periodic();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        DrivoTankWithPID driveTankWithPID = new DrivoTankWithPID(drivetrain, drivetrain.getLeftPIDSettings(),
                drivetrain.getRightPIDSettings(), set1, set2, drivetrain::getLeftEncoderPosition,
                drivetrain::getRightEncoderPosition);
        driveTankWithPID.schedule();
//        new lol().schedule();
//        command.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        command.cancel();
    }

    @Override
    public void teleopPeriodic() {
        double speed = ps.getRightY();
        double rotate = ps.getLeftX();
        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }

        if (Math.abs(rotate) < 0.05) {
            rotate = 0;
        }
        // drivetrain.setSpeeds((speed - rotate) * -0.4, (speed + rotate) * -0.4);
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
        drivetrain.simulationPeriodic();
    }
}
