package com.lcrobotics.easyftclib.tools.geometry;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d() {
        this(0, 0);
    }

    /**
     * Constructs a vector with given components
     *
     * @param x x component
     * @param y y component
     */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d(Vector2d other) {
        x = other.getX();
        y = other.getY();
    }

    public Vector2d rotateBy(double angle) {
        angle = Math.toRadians(angle);
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double x2 = cos * x - sin * y;
        double y2 = sin * x + cos * y;

        return new Vector2d(x2, y2);
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }
    /**
     * @return angle of the vector
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return plus(other.unaryMinus());
    }

    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }
    
    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }
    /**
     * @return x component of vector
     */
    public double getX() {
        return x;
    }
    /**
     * @return y component of vector
     */
    public double getY() {
        return y;
    }
}
