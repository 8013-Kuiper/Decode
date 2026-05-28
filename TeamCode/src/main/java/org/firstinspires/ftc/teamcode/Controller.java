package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private Gamepad gamepad;

    //Previous states
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

    //Pressed (edge detection)
    private boolean aPressed, bPressed, xPressed, yPressed;
    private boolean dpadUpPressed, dpadDownPressed, dpadLeftPressed, dpadRightPressed;

    //Toggles
    private boolean toggleA, toggleB, toggleX, toggleY;
    private boolean toggleDpadUp, toggleDpadDown, toggleDpadLeft, toggleDpadRight;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // Current states
        boolean currA = gamepad.a;
        boolean currB = gamepad.b;
        boolean currX = gamepad.x;
        boolean currY = gamepad.y;

        boolean currDpadUp = gamepad.dpad_up;
        boolean currDpadDown = gamepad.dpad_down;
        boolean currDpadLeft = gamepad.dpad_left;
        boolean currDpadRight = gamepad.dpad_right;

        //Edge detection
        aPressed = currA && !prevA;
        bPressed = currB && !prevB;
        xPressed = currX && !prevX;
        yPressed = currY && !prevY;

        dpadUpPressed = currDpadUp && !prevDpadUp;
        dpadDownPressed = currDpadDown && !prevDpadDown;
        dpadLeftPressed = currDpadLeft && !prevDpadLeft;
        dpadRightPressed = currDpadRight && !prevDpadRight;

        //Toggles
        if (aPressed)
            toggleA = !toggleA;
        if (bPressed)
            toggleB = !toggleB;
        if (xPressed)
            toggleX = !toggleX;
        if (yPressed)
            toggleY = !toggleY;

        if (dpadUpPressed)
            toggleDpadUp = !toggleDpadUp;
        if (dpadDownPressed)
            toggleDpadDown = !toggleDpadDown;
        if (dpadLeftPressed)
            toggleDpadLeft = !toggleDpadLeft;
        if (dpadRightPressed)
            toggleDpadRight = !toggleDpadRight;

        //Save previous
        prevA = currA;
        prevB = currB;
        prevX = currX;
        prevY = currY;

        prevDpadUp = currDpadUp;
        prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft;
        prevDpadRight = currDpadRight;
    }

    //Swerve Drive Specific Operations...
    public double getResultant() {
        double x = getLeftX();
        double y = getLeftY();
        return Math.min(Math.sqrt(x * x + y * y), 1.0);
    }

    //Angle in degrees [0, 360)
    boolean noInput;
    double angle;
    public double getAngle() {
        double y = -gamepad.left_stick_y; // invert
        double x = gamepad.left_stick_x;

        if (y == 0 && x == 0) {
            noInput = true;
            angle = 0.0;
        } else {
            noInput = false;
            angle = (Math.toDegrees(Math.atan2(y, -x)) - 90);
        }

        if (angle < 0) angle += 360;

        return angle;
    }
    public boolean isNoInput() {
        return noInput;
    }

    public double getRotation() {
        if (getRightX() != 0) {
            if (noInput) {
                noInput = false;
            }
        }
        return getRightX();
    }

    //Bumper Getters (RAW)
    public boolean getRightBumper() {
        return gamepad.right_bumper;
    }

    public boolean getLeftBumper() {
        return gamepad.left_bumper;
    }

    //Stick Getters (RAW)
    public double getLeftX() {
        return gamepad.left_stick_x;
    }
    public double getLeftY() {
        return -gamepad.left_stick_y;
    }
    public double getRightX() {
        return gamepad.right_stick_x;
    }
    public double getRightY() {
        return -gamepad.right_stick_y;
    }

    //Triggers (RAW)
    public double getLeftTrigger() {
        return gamepad.left_trigger;
    }
    public double getRightTrigger() {
        return gamepad.right_trigger;
    }

    //Buttons (RAW)
    public boolean getA() {
        return gamepad.a;
    }
    public boolean getB() {
        return gamepad.b;
    }
    public boolean getX() {
        return gamepad.x;
    }
    public boolean getY() {
        return gamepad.y;
    }

    public boolean getDpadUp() {
        return gamepad.dpad_up;
    }
    public boolean getDpadDown() {
        return gamepad.dpad_down;
    }
    public boolean getDpadLeft() {
        return gamepad.dpad_left;
    }
    public boolean getDpadRight() {
        return gamepad.dpad_right;
    }

    //Buttons (Pressed)
    //If a button is pressed, set it to true, but only once per time pressed
    public boolean getAPressed() {
        return aPressed;
    }
    public boolean getBPressed() {
        return bPressed;
    }
    public boolean getXPressed() {
        return xPressed;
    }
    public boolean getYPressed() {
        return yPressed;
    }

    public boolean getDpadUpPressed() {
        return dpadUpPressed;
    }
    public boolean getDpadDownPressed() {
        return dpadDownPressed;
    }
    public boolean getDpadLeftPressed() {
        return dpadLeftPressed;
    }
    public boolean getDpadRightPressed() {
        return dpadRightPressed;
    }

    //Button Toggles
    //If the button is pressed, set it to true, if pressed again, set it to false.
    public boolean getAToggle() {
        return toggleA;
    }
    public boolean getBToggle() {
        return toggleB;
    }
    public boolean getXToggle() {
        return toggleX;
    }
    public boolean getYToggle() {
        return toggleY;
    }

    public boolean getDpadUpToggle() {
        return toggleDpadUp;
    }
    public boolean getDpadDownToggle() {
        return toggleDpadDown;
    }
    public boolean getDpadLeftToggle() {
        return toggleDpadLeft;
    }
    public boolean getDpadRightToggle() {
        return toggleDpadRight;
    }
}