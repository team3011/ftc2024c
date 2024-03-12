package org.firstinspires.ftc.teamcode.Subsystems;

public class Point {

    int x;
    int y;

    public Point (int inX, int inY) {

        this.x = inX;
        this.y = inY;

    }

    public int returnX() {return this.x; }

    public int returnY() { return this.y; }

    public void setX(int input) { this.x = input; }

    public void setY(int input) { this.y = input; }

    public void setXY(int inputX, int inputY) {
        setX(inputX);
        setY(inputY);
    }

}
