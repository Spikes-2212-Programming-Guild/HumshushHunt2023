package frc.robot.commands.athena;

import java.time.Clock;

public class Group4 extends ProgramBase {

    @Override
    public void writeProgram() {

        moveForwardInSeconds(3);
        moveArm();
        releaseCube();
    }
}
