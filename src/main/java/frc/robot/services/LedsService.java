package frc.robot.services;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Gripper;

public class LedsService {

    private final Gripper gripper;

    private static final int NUMBER_OF_LEDS = 60;

    private static LedsService instance;

    private final VisionService vision;
    private LedsService(AddressableLED led, AddressableLEDBuffer ledBuffer, VisionService vision, Gripper gripper) {
        this.led = led;
        this.ledBuffer = ledBuffer;
        this.vision = vision;
        this.gripper = gripper;
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public static LedsService getInstance() {
        if (instance == null) {
            instance = new LedsService(new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS), VisionService.getInstance(), Gripper.getInstance());
        }
        return instance;
    }

    public void periodic() {
        boolean hasTarget = vision.limelightHasTarget();
        boolean holdingGamePiece = gripper.sensorHasTarget();

        if (!hasTarget && !holdingGamePiece) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, Mode.EMPTY_GRIPPER.red, Mode.EMPTY_GRIPPER.green, Mode.EMPTY_GRIPPER.blue);
            }
        }

        if (hasTarget && !holdingGamePiece) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, Mode.ALLIGNED.red, Mode.ALLIGNED.green, Mode.ALLIGNED.blue);
            }
        }

        if (!hasTarget && holdingGamePiece) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, Mode.HAS_GAME_PIECE.red, Mode.HAS_GAME_PIECE.green, Mode.HAS_GAME_PIECE.blue);
            }
        }

        if (hasTarget && holdingGamePiece) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, Mode.HAS_GAME_PIECE_AND_ALLIGNED.red, Mode.HAS_GAME_PIECE_AND_ALLIGNED.green,
                        Mode.HAS_GAME_PIECE_AND_ALLIGNED.blue);
            }
        }
    }

    public enum Mode {

        EMPTY_GRIPPER(254, 0, 0), ALLIGNED(204, 0, 254), HAS_GAME_PIECE(0, 0, 254),
        HAS_GAME_PIECE_AND_ALLIGNED(0, 254, 0);

        public final int red;
        public final int green;
        public final int blue;

        Mode(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }
}
