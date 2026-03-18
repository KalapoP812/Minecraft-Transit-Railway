/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.tool;

import org.mtr.core.tool.Utilities;

public class Vector {
    public final double x;
    public final double y;
    public final double z;

    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double x() {
        return this.x;
    }

    public double y() {
        return this.y;
    }

    public double z() {
        return this.z;
    }

    public Vector add(double x, double y, double z) {
        return new Vector(this.x + x, this.y + y, this.z + z);
    }

    public Vector add(Vector vector) {
        return this.add(vector.x, vector.y, vector.z);
    }

    public Vector multiply(double x, double y, double z) {
        return new Vector(this.x * x, this.y * y, this.z * z);
    }

    public Vector multiply(Vector vector) {
        return this.multiply(vector.x, vector.y, vector.z);
    }

    public Vector rotateX(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector(this.x, this.y * cos + this.z * sin, this.z * cos - this.y * sin);
    }

    public Vector rotateY(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector(this.x * cos + this.z * sin, this.y, this.z * cos - this.x * sin);
    }

    public Vector rotateZ(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector(this.x * cos + this.y * sin, this.y * cos - this.x * sin, this.z);
    }

    public double distanceTo(Vector vec) {
        double differenceX = vec.x - this.x;
        double differenceY = vec.y - this.y;
        double differenceZ = vec.z - this.z;
        return Math.sqrt(differenceX * differenceX + differenceY * differenceY + differenceZ * differenceZ);
    }

    public Vector normalize() {
        double length = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
        return length < 1.0E-4 ? new Vector(0.0, 0.0, 0.0) : new Vector(this.x / length, this.y / length, this.z / length);
    }

    public static Vector getAverage(Vector position1, Vector position2) {
        return new Vector(Utilities.getAverage(position1.x, position2.x), Utilities.getAverage(position1.y, position2.y), Utilities.getAverage(position1.z, position2.z));
    }
}

