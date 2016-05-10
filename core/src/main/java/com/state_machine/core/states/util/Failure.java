package com.state_machine.core.states.util;

import com.state_machine.core.states.util.ErrorType;
import org.ros.message.Time;

public class Failure {

    private ErrorType error;
    private Time time;

    public Failure(ErrorType error, Time time){
        this.error = error;
        this.time = time;
    }

    public ErrorType getError(){ return error; }
    public Time getTime(){ return time; }
    public String toString(){ return error.toString() + " at " + time.toString(); }
}
