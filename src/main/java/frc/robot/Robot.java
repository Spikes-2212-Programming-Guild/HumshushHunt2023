// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivotrain;

public class Robot extends TimedRobot {

    Drivotrain drivotrain = new Drivotrain();

    PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        drivotrain.periodic();
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

    }

    @Override
    public void teleopPeriodic() {
        double speed = xbox.getRightY();
        double rotate = xbox.getLeftX();
        if (Math.abs(speed) > 0.05 || Math.abs(rotate) > 0.05)
            drivotrain.setSpeeds(speed - rotate, speed + rotate);
        else {
            speed = 0;
            rotate = 0;
            drivotrain.setSpeeds(speed - rotate, speed + rotate);
        }
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
        drivotrain.simulationPeriodic();
    }
}
