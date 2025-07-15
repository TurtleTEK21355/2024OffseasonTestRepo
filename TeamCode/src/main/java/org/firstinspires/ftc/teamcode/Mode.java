package org.firstinspires.ftc.teamcode;

public class Mode {

    public enum State {UP,DOWN,MIDDLE}
    private State state = State.MIDDLE;
    private double value = 0;
    private String name = "unnamed";

    public Mode() {}
    public Mode(double value){
        this.value = value;
    }
    public Mode(String name){
        this.name = name;
    }
    public Mode(double value, String name){
        this.value = value;
        this.name = name;
    }

    /**
     * changes state to up or down
     * @param inputUp boolean that sets it to UP
     * @param inputDown boolean that sets it to DOWN
     */
    public void stateChange(boolean inputUp, boolean inputDown){
        if (inputUp){
            state = State.UP;
        }
        else if (inputDown){
            state = State.DOWN;
        }
    }

    /**
     * gets the current enum state
     * @return the current state
     */
    public State getState(){
        return state;
    }

    /**
     * sets the state to a enum
     * @param assignedState the new state
     */
    public void setState(State assignedState){
        state = assignedState;
    }

    public void valueChange(){
        if (state == State.UP){
            this.value += 0.01;
        } else if (state == State.DOWN){
            this.value -= 0.01;
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