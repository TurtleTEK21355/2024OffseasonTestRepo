package org.firstinspires.ftc.teamcode;

public class Mode {
    public enum State {UP,DOWN,MIDDLE}
    private State state = State.MIDDLE;
    private double value = 0;
    private String name;
    public Mode(){//no parameters
        value = 0;
    }
    public Mode(double value){//only value
    }
    public Mode(double value, String name){//value and string
    }

    public void stateChange(boolean inputUp, boolean inputDown){//changes state to up or down
        if (inputUp){
            State state = State.UP;
        }
        else if (inputDown){
            State state = State.DOWN;
        }
    }
    public State getState(){
        return state;
    }//gets the current enum state
    public void setState(State assignedState){//sets the state to a enum
        state = assignedState;
    }

    public void valueChange(){
        if (state == State.UP){
            value += 0.01;
        } else if (state == State.DOWN){
            value -= 0.01;
        }

        state = State.MIDDLE;
    }
    public void setName(String name){
        this.name = name;
    }
    public String getName(){
        return this.name;
    }
    public String getValue(){
        return Double.toString(this.value);
    }

}