package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

public class Leds extends DashboardedSubsystem {

    private final Namespace colorNamespace = namespace.addChild("led colors");

    public static final int NUMBER_OF_LEDS = 60;

    public static Leds instance;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private Leds(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        super("led");
        this.led = led;
        this.ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
        led.setLength(NUMBER_OF_LEDS);
        configureDashboard();
    }

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds("led", new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS));
        }
        return instance;
    }

    private void updateData() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    private void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }

    public void startLed() {
        led.start();
    }

    @Override
    public void configureDashboard() {
        namespace.addConstantInt("red v", red);
        namespace.addConstantInt("green v", green);
        namespace.addConstantInt("blue v", blue);
        namespace.putRunnable("update color", this::updateData);
        namespace.putRunnable("turn off", this::turnOff);
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
}
