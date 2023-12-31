package org.firstinspires.ftc.teamcode.Utility;

public class Vector2 {
    public double x;
    public double y;

    public Vector2 plus(Vector2 vec) {
        return new Vector2(this.x + vec.x, this.y + vec.y);
    }

    public Vector2 times(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }
    public Vector2 divide(double scalar) { return new Vector2(this.x / scalar, this.y / scalar); }
    public Vector2 minus(Vector2 vec) {
        return new Vector2(this.x - vec.x, this.y - vec.y);
    }

    public double getMagnitude() {//magnitude is the square root of the sum of the squares of x and y of vector
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public double dot(Vector2 vec) {
        return vec.x * vec.x + vec.y * vec.y;
    }

    public double dot(Vector2 vec1, Vector2 vec2) {
        return dot(vec1.minus(vec2));
    }

    public Vector2 getNormalized() {
        return new Vector2(this.x, this.y).divide(this.getMagnitude());
    }

    public Vector2 (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2() {
        this.x = 0;
        this.y = 0;
    }

    public String toString() {
        return String.format("(%.3f, %.3f)", this.x, this.y);
    }
}
