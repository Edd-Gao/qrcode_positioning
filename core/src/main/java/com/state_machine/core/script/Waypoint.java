package com.state_machine.core.script;

public class Waypoint {

    private final float x;
    private final float y;
    private final float z;

    public Waypoint(float x, float y, float z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }
}
