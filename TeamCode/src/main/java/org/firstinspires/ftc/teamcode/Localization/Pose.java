package org.firstinspires.ftc.teamcode.Localization;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.Utility.Vector2;

public class Pose {
    Vector2 position;
    double rotation;
    String label;

    public Pose(Vector2 position, double rotation) {
        this.position = position;
        this.rotation = rotation;
    }
    public Pose(String label, Vector2 position, double rotation) {
        this.position = position;
        this.rotation = rotation;
        this.label = label;
    }
    public Pose(double x, double y, double rotation) {
        this.position = new Vector2(x, y);
        this.rotation = rotation;
    }



    public boolean equals(Pose pose) {
        return this.position == pose.position && this.rotation == pose.rotation;
    }

    public String getLabel() {
        return label;
    }

}
