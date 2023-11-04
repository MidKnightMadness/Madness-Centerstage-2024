package org.firstinspires.ftc.teamcode.Utility;



public class ButtonToggle {
    boolean lastState;
    public boolean update(boolean currentState) {
        // if lastState is false
        if (currentState && !lastState) {
            lastState = true;
            return true;
        }

        lastState = false;

        return false;
    }

}
