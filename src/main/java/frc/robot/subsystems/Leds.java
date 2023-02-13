package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Leds extends DashboardedSubsystem {

    private final Namespace colorNamespace = namespace.addChild("led colors");

    public static final int NUMBER_OF_LEDS = 60;
    private Supplier<Integer> red = colorNamespace.addConstantInt("red value", 0);
    private Supplier<Integer> green = colorNamespace.addConstantInt("green value", 0);
    private Supplier<Integer> blue = colorNamespace.addConstantInt("blue value", 0);

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds("led", new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS));
        }
        return instance;
    }

    public static Leds instance;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private void updateData() {
        int rotateValue = 0; // here in case i decide to work on it

        for (int i = rotateValue; i < ledBuffer.getLength() + rotateValue; i++) {
            if (i % 12 == 3 || i % 12 == 6 || i % 12 == 8 || i % 12 == 11 || i % 12 == 0) {
                ledBuffer.setRGB(i % 60, 0, 0, 0);
            } else {
                ledBuffer.setRGB(i, red.get(), green.get(), blue.get());
            }
        }
        led.setData(ledBuffer);
    }

    private Leds(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        super("led");
        this.led = led;
        this.ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
        led.setLength(NUMBER_OF_LEDS);
        configureDashboard();
    }

    private enum colorPipeline {

        OFF(0, 0, 0, 0), CONE(1, 248, 255, 14), CUBE(2, 175, 14, 255);

        private int pipeline;

        private int red;
        private int green;
        private int blue;

        colorPipeline(int pipeline, int red, int green, int blue) {
            this.pipeline = pipeline;
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

    @Override
    public void configureDashboard() {
        namespace.putRunnable("update color", this::updateData);
        namespace.putRunnable("turn off", this::turnOff);
    }
}
