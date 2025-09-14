package org.firstinspires.ftc.teamcode.Holonomic;

/**
 * A simple class to represent a waypoint for autonomous navigation.
 * A waypoint includes a target x and y coordinate and a desired heading.
 */
public class Waypoint {
    public double x;
    public double y;
    public double heading; // The desired heading in degrees at this waypoint

    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}