package org.firstinspires.ftc.teamcode;

public class XyhVector {
    double x;
    double y;
    double h;

    public XyhVector(double x, double y, double h) {
        this.x=x;
        this.y=y;
        this.h=h;
    }

    public XyhVector(XyhVector start_pos){
        this.x= start_pos.x;
        this.y = start_pos.y;
        this.h = start_pos.h;
    }
}
