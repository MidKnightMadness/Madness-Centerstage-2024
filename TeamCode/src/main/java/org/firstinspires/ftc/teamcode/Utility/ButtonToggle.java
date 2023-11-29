package org.firstinspires.ftc.teamcode.Utility;



public class ButtonToggle {
    boolean lastState;
    boolean value;

    public ButtonToggle(){

    }
    public boolean update(boolean currentState) {
        // if lastState is false
        if (currentState && !lastState) {
            lastState = true;
            value = true;
            return true;
        }

        lastState = currentState;
        value = false;
        return false;
    }

    public boolean getValue() {
        return value;
    }

}
