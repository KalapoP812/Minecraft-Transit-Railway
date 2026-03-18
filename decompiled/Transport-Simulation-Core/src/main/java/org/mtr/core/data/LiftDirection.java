/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

public enum LiftDirection {
    NONE(0),
    UP(1),
    DOWN(-1);

    public final int sign;

    private LiftDirection(int sign) {
        this.sign = sign;
    }

    public static LiftDirection fromDifference(double difference) {
        if (difference == 0.0) {
            return NONE;
        }
        if (difference > 0.0) {
            return UP;
        }
        return DOWN;
    }
}

