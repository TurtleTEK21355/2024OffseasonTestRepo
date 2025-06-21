package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ModeController {
    private int mode = 0;
    private boolean previousItemLast = false;
    private boolean nextItemLast = false;
    List<Mode> modes = new ArrayList<>();
    private ElapsedTime elapsedTime;

    public ModeController(){
        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

    }

    public void add(Mode m){
        this.modes.add(m);
    }

    public void add(Mode... modes){
        this.modes.addAll(Arrays.asList(modes));
    }

    public String getCurrentModeName(){
        return modes.get(mode).getName();
    }

    public void modeSelection(boolean previousMode, boolean nextMode, boolean valueUp, boolean valueDown) {
        if (previousMode && !previousItemLast) {
            mode -= 1;
            previousItemLast = true;
        } else if (!previousMode) {
            previousItemLast = false;
        }

        if (nextMode && !nextItemLast) {
            mode += 1;
            nextItemLast = true;
        } else if (!nextMode) {
            nextItemLast = false;
        }

        if (mode > modes.size() - 1) {
            mode = 0;
        } else if (mode < 0) {
            mode = modes.size() - 1;
        }

        modes.get(mode).stateChange(valueUp, valueDown);


        if (elapsedTime.milliseconds() > 500){
            modes.get(mode).valueChange();

            elapsedTime.reset();
        }
    }
    public String reportModeValue(){
        String name = "";
        for(int i = 0; i < modes.size(); i++) {
            if (i == mode) {
                name = name.concat("> ");
            }
            name = name.concat(modes.get(i).getName() + " = " + modes.get(i).getValue()) + "\n";

        }
        return name;
    }
}
