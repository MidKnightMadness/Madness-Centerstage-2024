package org.firstinspires.ftc.teamcode.Utility;

public class Vector2<D extends Double> {
    public double x;
    public double y;

    public Vector2<Double> plus(Vector2<Double> vec) {
        return new Vector2<Double>(this.x + vec.x, this.y + vec.y);
    }

    public Vector2<Double> times(double scalar) {
        return new Vector2<Double>(this.x * scalar, this.y * scalar);
    }
    public Vector2<Double> divide(double scalar) { return new Vector2<Double>(this.x / scalar, this.y / scalar); }
    public Vector2<Double> minus(Vector2<Double> vec) {
        return new Vector2<Double>(this.x - vec.x, this.y - vec.y);
    }

    public double getMagnitude() {//magnitude is the square root of the sum of the squares of x and y of vector
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public double dot(Vector2<Double> vec) {
        return vec.x * vec.x + vec.y * vec.y;
    }

    public double dot(Vector2<Double> vec1, Vector2<Double> vec2) {
        return dot(vec1.minus(vec2));
    }

    public Vector2<Double> getNormalized() {
        return new Vector2<Double>(this.x, this.y).divide(this.getMagnitude());
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
