package lib.BlueShift.control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController {
    public enum CustomControllerType {
        XBOX,
        PS4,
        PS5
    }

    public enum CustomJoystickCurve {
        LINEAR,
        CUBIC
    }

    CustomControllerType type;
    int port;
    int curve;
    PS5Controller ps5Controller;
    PS4Controller ps4Controller;
    XboxController xboxController;

    /**
     * Creates a CustomController instance, lets you specify the controller this.type and the joystick curve
     * @param ContollerPort The port the controller is plugged into
     * @param this.type The this.type of controller
     * @param curve The curve of the joystick
     */
    public CustomController(int ContollerPort, CustomControllerType type, CustomJoystickCurve curve) {
        this.port = ContollerPort;
        this.type = type;

        switch (this.type) {
            case XBOX:
                xboxController = new XboxController(this.port);
                break;
            case PS4:
                ps4Controller = new PS4Controller(this.port);
                break;
            case PS5:
                ps5Controller = new PS5Controller(this.port);
                break;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }

        switch (curve) {
            case LINEAR:
                this.curve = 1;
                break;
            case CUBIC:
                this.curve = 3;
                break;
            default:
                throw new IllegalArgumentException("Invalid Joystick Curve");
        }
    }

    /**
     * Creates a CustomController instance, lets you specify the controller this.type
     * @param ContollerPort The port the controller is plugged into
     * @param this.type The this.type of controller
     */
    public CustomController(int ContollerPort, CustomControllerType type) {
        this.port = ContollerPort;
        this.type = type;
        this.curve = 1;

        switch (this.type) {
            case XBOX:
                xboxController = new XboxController(port);
                break;
            case PS4:
                ps4Controller = new PS4Controller(port);
                break;
            case PS5:
                ps5Controller = new PS5Controller(port);
                break;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the "X" axis value of the left stick
     * @return The axis value of the stick
     */
    public double getLeftY() {
        switch (this.type) {
            case XBOX:
                return Math.pow(xboxController.getLeftY(), this.curve)+0.0;
            case PS4:
                return Math.pow(ps4Controller.getLeftY(), this.curve)+0.0;
            case PS5:
                return Math.pow(ps5Controller.getLeftY(), this.curve)+0.0;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the "Y" axis value of the left stick
     * @return The axis value of the stick
     */
    public double getLeftX() {
        switch (this.type) {
            case XBOX:
                return Math.pow(xboxController.getLeftX(), this.curve)+0.0;
            case PS4:
                return Math.pow(ps4Controller.getLeftX(), this.curve)+0.0;
            case PS5:
                return Math.pow(ps5Controller.getLeftX(), this.curve)+0.0;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the "X" axis value of the right stick
     * @return The axis value of the stick
     */
    public double getRightX() {
        switch (this.type) {
            case XBOX:
                return Math.pow(xboxController.getRightX(), this.curve)+0.0;
            case PS4:
                return Math.pow(ps4Controller.getRightX(), this.curve)+0.0;
            case PS5:
                return Math.pow(ps5Controller.getRightX(), this.curve)+0.0;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the "Y" axis value of the right stick
     * @return The axis value of the stick
     */
    public double getRightY() {
        switch (this.type) {
            case XBOX:
                return Math.pow(xboxController.getRightY(), this.curve)+0.0;
            case PS4:
                return Math.pow(ps4Controller.getRightY(), this.curve)+0.0;
            case PS5:
                return Math.pow(ps5Controller.getRightY(), this.curve)+0.0;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the axis value of the Left Trigger
     * @return
     */
    public double getLeftTrigger() {
        switch (this.type) {
            case XBOX:
                return xboxController.getLeftTriggerAxis();
            case PS4:
                return ps4Controller.getL2Axis();
            case PS5:
                return ps5Controller.getL2Axis();
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Get the axis value of the Right Trigger
     * @return
     */
    public double getRightTrigger() {
        switch (this.type) {
            case XBOX:
                return xboxController.getRightTriggerAxis();
            case PS4:
                return ps4Controller.getR2Axis();
            case PS5:
                return ps5Controller.getR2Axis();
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "A" button or the "Cross" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger bottomButton() {
        return bottomButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "A" button or the "Cross" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger bottomButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.a(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.cross(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.cross(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "B" button or the "Circle" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightButton() {
        return rightButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "B" button or the "Circle" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.b(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.circle(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.circle(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "X" button or the "Square" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftButton() {
        return leftButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "X" button or the "Square" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.x(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.square(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.square(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Y" button or the "Triangle" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger topButton() {
        return topButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Y" button or the "Triangle" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger topButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.y(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.triangle(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.triangle(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Left Bumper" button or the "L1" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Left Bumper" button or the "L1" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftBumper(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.leftBumper(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.L1(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.L1(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Right Bumper" button or the "R1" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Right Bumper" button or the "R1" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightBumper(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.rightBumper(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.R1(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.R1(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Left Trigger" button or the "L2" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftTrigger() {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Left Trigger" button or the "L2" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger leftTrigger(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.leftTrigger(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.L2(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.L2(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Right Trigger" button or the "R2" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightTrigger() {
        return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Right Trigger" button or the "R2" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger rightTrigger(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.rightTrigger(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.R2(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.R2(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Pov Up" button
     * @return The Trigger instance of the button
     */
    public Trigger povUp() {
        return povUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Pov Up" button
     * @return The Trigger instance of the button
     */
    public Trigger povUp(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.povUp(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.povUp(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.povUp(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Pov Down" button
     * @return The Trigger instance of the button
     */
    public Trigger povDown() {
        return povDown(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Pov Down" button
     * @return The Trigger instance of the button
     */
    public Trigger povDown(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.povDown(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.povDown(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.povDown(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Pov Left" button
     * @return The Trigger instance of the button
     */
    public Trigger povLeft() {
        return povLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Pov Left" button
     * @return The Trigger instance of the button
     */
    public Trigger povLeft(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.povLeft(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.povLeft(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.povLeft(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Pov Right" button
     * @return The Trigger instance of the button
     */
    public Trigger povRight() {
        return povRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Pov Right" button
     * @return The Trigger instance of the button
     */
    public Trigger povRight(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.povRight(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.povRight(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.povRight(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the Left Stick button
     * @return The Trigger instance of the button
     */
    public Trigger leftStickButton() {
        return leftStickButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the Left Stick button
     * @return The Trigger instance of the button
     */
    public Trigger leftStickButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.leftStick(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.L3(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.L3(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the Right Stick button
     * @return The Trigger instance of the button
     */
    public Trigger rightStickButton() {
        return rightStickButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the Right Stick button
     * @return The Trigger instance of the button
     */
    public Trigger rightStickButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.rightStick(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.R3(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.R3(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Options" button or the "Start" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger startButton() {
        return startButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Options" button or the "Start" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger startButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.start(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.options(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.options(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the "Create" button or the "Back" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger backButton() {
        return backButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the "Create" button or the "Back" button depending on the controller this.type
     * @return The Trigger instance of the button
     */
    public Trigger backButton(EventLoop loop) {
        switch (this.type) {
            case XBOX:
                return this.xboxController.back(loop).castTo(Trigger::new);
            case PS4:
                return this.ps4Controller.share(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.create(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Constructs a new Trigger instance for the Touchpad button
     * @return The Trigger instance of the button
     */
    public Trigger homeButton() {
        return homeButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a new Trigger instance for the Touchpad button
     * @return The Trigger instance of the button
     */
    public Trigger homeButton(EventLoop loop) {
        switch (this.type) {
            case PS4:
                return this.ps4Controller.touchpad(loop).castTo(Trigger::new);
            case PS5:
                return this.ps5Controller.touchpad(loop).castTo(Trigger::new);
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Set the left rumble of the controller
     * @param value The value of the rumble
     */
    public void setLeftRumble(double value) {
        switch (this.type) {
            case XBOX:
                this.xboxController.setRumble(RumbleType.kLeftRumble, value);
                break;
            case PS4:
                this.ps4Controller.setRumble(RumbleType.kLeftRumble, value);
                break;
            case PS5:
                this.ps5Controller.setRumble(RumbleType.kLeftRumble, value);
                break;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Set the right rumble of the controller
     * @param value The value of the rumble
     */
    public void setRightRumble(double value) {
        switch (this.type) {
            case XBOX:
                this.xboxController.setRumble(RumbleType.kRightRumble, value);
                break;
            case PS4:
                this.ps4Controller.setRumble(RumbleType.kRightRumble, value);
                break;
            case PS5:
                this.ps5Controller.setRumble(RumbleType.kRightRumble, value);
                break;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }

    /**
     * Set the rumble of the controller
     * @param rumbleValue The value of the rumble
     */
    public void setRumble(double rumbleValue) {
        setRumble(rumbleValue, rumbleValue);
    }

    /**
     * Set the rumble of the controller
     * @param leftValue The value of the left rumble
     * @param rightValue The value of the right rumble
     */
    public void setRumble(double leftValue, double rightValue) {
        switch (this.type) {
            case XBOX:
                this.xboxController.setRumble(RumbleType.kLeftRumble, leftValue);
                this.xboxController.setRumble(RumbleType.kRightRumble, rightValue);
                break;
            case PS4:
                this.ps4Controller.setRumble(RumbleType.kLeftRumble, leftValue);
                this.ps4Controller.setRumble(RumbleType.kRightRumble, rightValue);
                break;
            case PS5:
                this.ps5Controller.setRumble(RumbleType.kLeftRumble, leftValue);
                this.ps5Controller.setRumble(RumbleType.kRightRumble, rightValue);
                break;
            default:
                throw new IllegalArgumentException("Invalid Controller Type");
        }
    }
}
