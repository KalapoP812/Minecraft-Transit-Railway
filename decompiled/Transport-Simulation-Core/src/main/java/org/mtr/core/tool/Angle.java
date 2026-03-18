/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.tool;

public enum Angle {
    E(0.0f),
    SEE(22.5f),
    SE(45.0f),
    SSE(67.5f),
    S(90.0f),
    SSW(112.5f),
    SW(135.0f),
    SWW(157.5f),
    W(180.0f),
    NWW(202.5f),
    NW(225.0f),
    NNW(247.5f),
    N(270.0f),
    NNE(292.5f),
    NE(315.0f),
    NEE(337.5f);

    public final float angleDegrees;
    public final double angleRadians;
    public final double sin;
    public final double cos;
    public final double tan;
    public final double halfTan;
    private static final int DEGREES_IN_CIRCLE = 360;
    private static final int QUADRANTS;
    private static final float ANGLE_INCREMENT;

    private Angle(float angleDegrees) {
        this.angleDegrees = Angle.normalizeAngle(angleDegrees);
        this.angleRadians = Math.toRadians(this.angleDegrees);
        this.sin = Math.sin(this.angleRadians);
        this.cos = Math.cos(this.angleRadians);
        this.tan = Math.tan(this.angleRadians);
        this.halfTan = Math.tan(this.angleRadians / 2.0);
    }

    public Angle getOpposite() {
        switch (this) {
            default: {
                return W;
            }
            case SEE: {
                return NWW;
            }
            case SE: {
                return NW;
            }
            case SSE: {
                return NNW;
            }
            case S: {
                return N;
            }
            case SSW: {
                return NNE;
            }
            case SW: {
                return NE;
            }
            case SWW: {
                return NEE;
            }
            case W: {
                return E;
            }
            case NWW: {
                return SEE;
            }
            case NW: {
                return SE;
            }
            case NNW: {
                return SSE;
            }
            case N: {
                return S;
            }
            case NNE: {
                return SSW;
            }
            case NE: {
                return SW;
            }
            case NEE: 
        }
        return SWW;
    }

    public Angle getClosest45() {
        switch (this) {
            default: {
                return this;
            }
            case NNW: 
            case NNE: {
                return N;
            }
            case SEE: 
            case NEE: {
                return E;
            }
            case SSE: 
            case SSW: {
                return S;
            }
            case SWW: 
            case NWW: 
        }
        return W;
    }

    public Angle add(Angle angle) {
        return Angle.fromAngle(this.angleDegrees + angle.angleDegrees);
    }

    public Angle sub(Angle angle) {
        return Angle.fromAngle(this.angleDegrees - angle.angleDegrees);
    }

    public boolean isParallel(Angle angle) {
        return this == angle || this == angle.getOpposite();
    }

    public boolean similarFacing(float newAngleDegrees) {
        return Angle.similarFacing(this.angleDegrees, newAngleDegrees);
    }

    public static boolean similarFacing(float angleDegrees1, float angleDegrees2) {
        return Math.abs(Angle.normalizeAngle(angleDegrees1 - angleDegrees2)) < 90.0f;
    }

    public static int getQuadrant(float angleDegrees, boolean include225) {
        int factor = include225 ? 1 : 2;
        return Math.round((Angle.normalizeAngle(angleDegrees) + 360.0f) / ANGLE_INCREMENT / (float)factor) % (QUADRANTS / factor);
    }

    public static Angle fromAngle(float angleDegrees) {
        return Angle.values()[Angle.getQuadrant(angleDegrees, true)];
    }

    private static float normalizeAngle(float angleDegrees) {
        int additional = 0;
        while (angleDegrees + (float)additional < -180.0f) {
            additional += 360;
        }
        while (angleDegrees + (float)additional >= 180.0f) {
            additional -= 360;
        }
        return angleDegrees + (float)additional;
    }

    static {
        QUADRANTS = Angle.values().length;
        ANGLE_INCREMENT = 360.0f / (float)QUADRANTS;
    }
}

