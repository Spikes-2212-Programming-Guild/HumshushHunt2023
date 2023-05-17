package frc.robot.commands.athena;

public class Group4 extends ProgramBase {

    @Override
    public void writeProgram() {
        moveForwardInSeconds(3);
        moveBackwardInSeconds(3);
        moveForwardInSeconds(3);
        moveBackwardInSeconds(3);
        moveForwardInSeconds(3);
          moveArm();
          releaseCube();
        moveBackwardInSeconds(7);
        }
}
