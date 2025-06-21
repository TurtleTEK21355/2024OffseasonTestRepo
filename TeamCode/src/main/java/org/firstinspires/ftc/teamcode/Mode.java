package org.firstinspires.ftc.teamcode;

public class Mode {
    enum State {UP,DOWN,MIDDLE}

    private void stateChange(boolean inputUp, boolean inputDown){
        if (inputUp){
            State state = State.UP;
        }
        else if (inputDown){
            State state = State.DOWN;
        }
    }

}