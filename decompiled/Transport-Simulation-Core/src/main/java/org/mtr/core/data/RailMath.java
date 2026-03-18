/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleDoubleImmutablePair
 */
package org.mtr.core.data;

import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.libraries.it.unimi.dsi.fastutil.doubles.DoubleDoubleImmutablePair;

public class RailMath {
    public final long minX;
    public final long minY;
    public final long minZ;
    public final long maxY;
    public final long maxX;
    public final long maxZ;
    private final Rail.Shape shape;
    private final double verticalRadius;
    private final double h1;
    private final double k1;
    private final double h2;
    private final double k2;
    private final double r1;
    private final double r2;
    private final double tStart1;
    private final double tEnd1;
    private final double tStart2;
    private final double tEnd2;
    private final long yStart;
    private final long yEnd;
    private final boolean reverseT1;
    private final boolean reverseT2;
    private final boolean isStraight1;
    private final boolean isStraight2;
    private static final double ACCEPT_THRESHOLD = 1.0E-4;
    private static final int CABLE_CURVATURE_SCALE = 1000;
    private static final int MAX_CABLE_DIP = 8;

    public RailMath(Position position1, Angle angle1, Position position2, Angle angle2, Rail.Shape shape, double verticalRadius) {
        long xStart = position1.getX();
        long zStart = position1.getZ();
        long xEnd = position2.getX();
        long zEnd = position2.getZ();
        Vector vecDifference = new Vector(position2.getX() - position1.getX(), 0.0, position2.getZ() - position1.getZ());
        Vector vecDifferenceRotated = vecDifference.rotateY((float)angle1.angleRadians);
        double deltaForward = vecDifferenceRotated.z();
        double deltaSide = vecDifferenceRotated.x();
        if (angle1.isParallel(angle2)) {
            if (Math.abs(deltaForward) < 1.0E-4) {
                this.h1 = angle1.cos;
                this.k1 = angle1.sin;
                if (Math.abs(this.h1) >= 0.5 && Math.abs(this.k1) >= 0.5) {
                    this.r1 = (this.h1 * (double)zStart - this.k1 * (double)xStart) / this.h1 / this.h1;
                    this.tStart1 = (double)xStart / this.h1;
                    this.tEnd1 = (double)xEnd / this.h1;
                } else {
                    double div = angle1.add((Angle)angle1).cos;
                    this.r1 = (this.h1 * (double)zStart - this.k1 * (double)xStart) / div;
                    this.tStart1 = (this.h1 * (double)xStart - this.k1 * (double)zStart) / div;
                    this.tEnd1 = (this.h1 * (double)xEnd - this.k1 * (double)zEnd) / div;
                }
                this.r2 = 0.0;
                this.k2 = 0.0;
                this.h2 = 0.0;
                this.reverseT1 = this.tStart1 > this.tEnd1;
                this.reverseT2 = false;
                this.isStraight2 = true;
                this.isStraight1 = true;
                this.tEnd2 = 0.0;
                this.tStart2 = 0.0;
            } else if (Math.abs(deltaSide) > 1.0E-4) {
                double radius = (deltaForward * deltaForward + deltaSide * deltaSide) / (4.0 * deltaForward);
                this.r1 = this.r2 = Math.abs(radius);
                this.h1 = (double)xStart - radius * angle1.sin;
                this.k1 = (double)zStart + radius * angle1.cos;
                this.h2 = (double)xEnd - radius * angle2.sin;
                this.k2 = (double)zEnd + radius * angle2.cos;
                this.reverseT1 = deltaForward < 0.0 != deltaSide < 0.0;
                this.reverseT2 = !this.reverseT1;
                this.tStart1 = RailMath.getTBounds(xStart, this.h1, zStart, this.k1, this.r1);
                this.tEnd1 = RailMath.getTBounds((double)xStart + vecDifference.x() / 2.0, this.h1, (double)zStart + vecDifference.z() / 2.0, this.k1, this.r1, this.tStart1, this.reverseT1);
                this.tStart2 = RailMath.getTBounds((double)xStart + vecDifference.x() / 2.0, this.h2, (double)zStart + vecDifference.z() / 2.0, this.k2, this.r2);
                this.tEnd2 = RailMath.getTBounds(xEnd, this.h2, zEnd, this.k2, this.r2, this.tStart2, this.reverseT2);
                this.isStraight2 = false;
                this.isStraight1 = false;
            } else {
                this.r2 = 0.0;
                this.r1 = 0.0;
                this.k2 = 0.0;
                this.h2 = 0.0;
                this.k1 = 0.0;
                this.h1 = 0.0;
                this.tEnd2 = 0.0;
                this.tEnd1 = 0.0;
                this.tStart2 = 0.0;
                this.tStart1 = 0.0;
                this.reverseT1 = false;
                this.reverseT2 = false;
                this.isStraight2 = true;
                this.isStraight1 = true;
            }
        } else {
            Angle newAngle1 = vecDifferenceRotated.x() < -1.0E-4 ? angle1.getOpposite() : angle1;
            Angle newAngle2 = angle2.cos * vecDifference.x() + angle2.sin * vecDifference.z() < -1.0E-4 ? angle2.getOpposite() : angle2;
            double angleForward = Math.atan2(deltaForward, deltaSide);
            Angle railAngleDifference = newAngle2.sub(newAngle1);
            double angleDifference = railAngleDifference.angleRadians;
            if (Math.signum(angleForward) == Math.signum(angleDifference)) {
                double absAngleForward = Math.abs(angleForward);
                if (absAngleForward - Math.abs(angleDifference / 2.0) < 1.0E-4) {
                    double offsetSide = Math.abs(deltaForward / railAngleDifference.halfTan);
                    double remainingSide = deltaSide - offsetSide;
                    double deltaXEnd = (double)xStart + remainingSide * newAngle1.cos;
                    double deltaZEnd = (double)zStart + remainingSide * newAngle1.sin;
                    this.h1 = newAngle1.cos;
                    this.k1 = newAngle1.sin;
                    if (Math.abs(this.h1) >= 0.5 && Math.abs(this.k1) >= 0.5) {
                        this.r1 = (this.h1 * (double)zStart - this.k1 * (double)xStart) / this.h1 / this.h1;
                        this.tStart1 = (double)xStart / this.h1;
                        this.tEnd1 = deltaXEnd / this.h1;
                    } else {
                        double div = newAngle1.add((Angle)newAngle1).cos;
                        this.r1 = (this.h1 * (double)zStart - this.k1 * (double)xStart) / div;
                        this.tStart1 = (this.h1 * (double)xStart - this.k1 * (double)zStart) / div;
                        this.tEnd1 = (this.h1 * deltaXEnd - this.k1 * deltaZEnd) / div;
                    }
                    this.isStraight1 = true;
                    this.reverseT1 = this.tStart1 > this.tEnd1;
                    double radius = deltaForward / (1.0 - railAngleDifference.cos);
                    this.r2 = Math.abs(radius);
                    this.h2 = deltaXEnd - radius * newAngle1.sin;
                    this.k2 = deltaZEnd + radius * newAngle1.cos;
                    this.reverseT2 = deltaForward < 0.0;
                    this.tStart2 = RailMath.getTBounds(deltaXEnd, this.h2, deltaZEnd, this.k2, this.r2);
                    this.tEnd2 = RailMath.getTBounds(xEnd, this.h2, zEnd, this.k2, this.r2, this.tStart2, this.reverseT2);
                    this.isStraight2 = false;
                } else if (absAngleForward - Math.abs(angleDifference) < 1.0E-4) {
                    double crossSide = deltaForward / railAngleDifference.tan;
                    double remainingSide = (deltaSide - crossSide) * (1.0 + railAngleDifference.cos);
                    double remainingForward = (deltaSide - crossSide) * railAngleDifference.sin;
                    double deltaXEnd = (double)xStart + remainingSide * newAngle1.cos - remainingForward * newAngle1.sin;
                    double deltaZEnd = (double)zStart + remainingSide * newAngle1.sin + remainingForward * newAngle1.cos;
                    double radius = (deltaSide - deltaForward / railAngleDifference.tan) / railAngleDifference.halfTan;
                    this.r1 = Math.abs(radius);
                    this.h1 = (double)xStart - radius * newAngle1.sin;
                    this.k1 = (double)zStart + radius * newAngle1.cos;
                    this.isStraight1 = false;
                    this.reverseT1 = deltaForward < 0.0;
                    this.tStart1 = RailMath.getTBounds(xStart, this.h1, zStart, this.k1, this.r1);
                    this.tEnd1 = RailMath.getTBounds(deltaXEnd, this.h1, deltaZEnd, this.k1, this.r1, this.tStart1, this.reverseT1);
                    this.h2 = newAngle2.cos;
                    this.k2 = newAngle2.sin;
                    if (Math.abs(this.h2) >= 0.5 && Math.abs(this.k2) >= 0.5) {
                        this.r2 = (this.h2 * deltaZEnd - this.k2 * deltaXEnd) / this.h2 / this.h2;
                        this.tStart2 = deltaXEnd / this.h2;
                        this.tEnd2 = (double)xEnd / this.h2;
                    } else {
                        double div = newAngle2.add((Angle)newAngle2).cos;
                        this.r2 = (this.h2 * deltaZEnd - this.k2 * deltaXEnd) / div;
                        this.tStart2 = (this.h2 * deltaXEnd - this.k2 * deltaZEnd) / div;
                        this.tEnd2 = (this.h2 * (double)xEnd - this.k2 * (double)zEnd) / div;
                    }
                    this.isStraight2 = true;
                    this.reverseT2 = this.tStart2 > this.tEnd2;
                } else {
                    this.r2 = 0.0;
                    this.r1 = 0.0;
                    this.k2 = 0.0;
                    this.h2 = 0.0;
                    this.k1 = 0.0;
                    this.h1 = 0.0;
                    this.tEnd2 = 0.0;
                    this.tEnd1 = 0.0;
                    this.tStart2 = 0.0;
                    this.tStart1 = 0.0;
                    this.reverseT1 = false;
                    this.reverseT2 = false;
                    this.isStraight2 = true;
                    this.isStraight1 = true;
                }
            } else {
                this.r2 = 0.0;
                this.r1 = 0.0;
                this.k2 = 0.0;
                this.h2 = 0.0;
                this.k1 = 0.0;
                this.h1 = 0.0;
                this.tEnd2 = 0.0;
                this.tEnd1 = 0.0;
                this.tStart2 = 0.0;
                this.tStart1 = 0.0;
                this.reverseT1 = false;
                this.reverseT2 = false;
                this.isStraight2 = true;
                this.isStraight1 = true;
            }
        }
        this.yStart = position1.getY();
        this.yEnd = position2.getY();
        this.shape = shape;
        this.verticalRadius = Math.min(verticalRadius, this.getMaxVerticalRadius());
        double[] bounds = new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, -1.7976931348623157E308, -1.7976931348623157E308, -1.7976931348623157E308};
        this.render((x1, z1, x2, z2, x3, z3, x4, z4, y1, y2) -> {
            bounds[0] = Math.min(x1, bounds[0]);
            bounds[0] = Math.min(x2, bounds[0]);
            bounds[0] = Math.min(x3, bounds[0]);
            bounds[0] = Math.min(x4, bounds[0]);
            bounds[1] = Math.min(y1, bounds[1]);
            bounds[1] = Math.min(y2, bounds[1]);
            bounds[2] = Math.min(z1, bounds[2]);
            bounds[2] = Math.min(z2, bounds[2]);
            bounds[2] = Math.min(z3, bounds[2]);
            bounds[2] = Math.min(z4, bounds[2]);
            bounds[3] = Math.max(x1, bounds[3]);
            bounds[3] = Math.max(x2, bounds[3]);
            bounds[3] = Math.max(x3, bounds[3]);
            bounds[3] = Math.max(x4, bounds[3]);
            bounds[4] = Math.max(y1, bounds[4]);
            bounds[4] = Math.max(y2, bounds[4]);
            bounds[5] = Math.max(z1, bounds[5]);
            bounds[5] = Math.max(z2, bounds[5]);
            bounds[5] = Math.max(z3, bounds[5]);
            bounds[5] = Math.max(z4, bounds[5]);
        }, 0.1, 0.0f, 0.0f);
        this.minX = bounds[0] > bounds[3] ? 0L : (long)Math.floor(bounds[0]);
        this.minY = bounds[1] > bounds[4] ? 0L : (long)Math.floor(bounds[1]);
        this.minZ = bounds[2] > bounds[5] ? 0L : (long)Math.floor(bounds[2]);
        this.maxX = bounds[3] < bounds[0] ? 0L : (long)Math.ceil(bounds[3]);
        this.maxY = bounds[4] < bounds[1] ? 0L : (long)Math.ceil(bounds[4]);
        this.maxZ = bounds[5] < bounds[2] ? 0L : (long)Math.ceil(bounds[5]);
    }

    public Vector getPosition(double rawValue, boolean reverse) {
        double count1 = Math.abs(this.tEnd1 - this.tStart1);
        double count2 = Math.abs(this.tEnd2 - this.tStart2);
        double clampedValue = Utilities.clamp(rawValue, 0.0, count1 + count2);
        double value = reverse ? count1 + count2 - clampedValue : clampedValue;
        double y = this.getPositionY(value);
        if (value <= count1) {
            return RailMath.getPositionXZ(this.h1, this.k1, this.r1, (double)(this.reverseT1 ? -1 : 1) * value + this.tStart1, 0.0, this.isStraight1).add(0.0, y, 0.0);
        }
        return RailMath.getPositionXZ(this.h2, this.k2, this.r2, (double)(this.reverseT2 ? -1 : 1) * (value - count1) + this.tStart2, 0.0, this.isStraight2).add(0.0, y, 0.0);
    }

    public double getLength() {
        return Math.abs(this.tEnd2 - this.tStart2) + Math.abs(this.tEnd1 - this.tStart1);
    }

    public Rail.Shape getShape() {
        return this.shape;
    }

    public DoubleDoubleImmutablePair getHorizontalRadii() {
        return new DoubleDoubleImmutablePair(this.isStraight1 ? 0.0 : Math.abs(this.r1), this.isStraight2 ? 0.0 : Math.abs(this.r2));
    }

    public double getVerticalRadius() {
        return this.verticalRadius;
    }

    public double getMaxVerticalRadius() {
        double length = this.getLength();
        double height = this.yEnd - this.yStart;
        return Math.floor((length * length + height * height) * 100.0 / Math.abs(4.0 * height)) / 100.0;
    }

    public void render(RenderRail callback, double interval, float offsetRadius1, float offsetRadius2) {
        this.renderSegment(this.h1, this.k1, this.r1, this.tStart1, this.tEnd1, 0.0, interval, offsetRadius1, offsetRadius2, this.reverseT1, this.isStraight1, callback);
        this.renderSegment(this.h2, this.k2, this.r2, this.tStart2, this.tEnd2, Math.abs(this.tEnd1 - this.tStart1), interval, offsetRadius1, offsetRadius2, this.reverseT2, this.isStraight2, callback);
    }

    boolean isValid() {
        return this.h1 != 0.0 || this.k1 != 0.0 || this.h2 != 0.0 || this.k2 != 0.0 || this.r1 != 0.0 || this.r2 != 0.0 || this.tStart1 != 0.0 || this.tStart2 != 0.0 || this.tEnd1 != 0.0 || this.tEnd2 != 0.0;
    }

    private void renderSegment(double h, double k, double r, double tStart, double tEnd, double rawValueOffset, double interval, float offsetRadius1, float offsetRadius2, boolean reverseT, boolean isStraight, RenderRail callback) {
        double count = Math.abs(tEnd - tStart);
        double increment = count < 0.5 || interval <= 0.0 ? 0.5 : count / (double)Math.round(count) * interval;
        Vector previousCorner1 = null;
        Vector previousCorner2 = null;
        double previousY = 0.0;
        for (double i = 0.0; i < count + increment - 0.1; i += increment) {
            double t = (double)(reverseT ? -1 : 1) * i + tStart;
            Vector corner1 = RailMath.getPositionXZ(h, k, r, t, offsetRadius2, isStraight);
            Vector corner2 = offsetRadius2 == offsetRadius1 ? corner1 : RailMath.getPositionXZ(h, k, r, t, offsetRadius1, isStraight);
            double y = this.getPositionY(i + rawValueOffset);
            if (previousCorner1 != null) {
                callback.renderRail(previousCorner1.x(), previousCorner1.z(), previousCorner2.x(), previousCorner2.z(), corner1.x(), corner1.z(), corner2.x(), corner2.z(), previousY, y);
            }
            previousCorner1 = corner2;
            previousCorner2 = corner1;
            previousY = y;
        }
    }

    private double getPositionY(double value) {
        double offsetValue;
        double yInitial;
        double yChange;
        if (this.yStart == this.yEnd) {
            return this.yStart;
        }
        double length = this.getLength();
        switch (this.shape) {
            case TWO_RADII: {
                int sign;
                if (this.verticalRadius <= 0.0) {
                    return value / length * (double)(this.yEnd - this.yStart) + (double)this.yStart;
                }
                double vTheta = this.getVTheta();
                double curveLength = Math.sin(vTheta) * this.verticalRadius;
                double curveHeight = (1.0 - Math.cos(vTheta)) * this.verticalRadius;
                int n = sign = this.yStart < this.yEnd ? 1 : -1;
                if (value < curveLength) {
                    return (double)sign * (this.verticalRadius - Math.sqrt(this.verticalRadius * this.verticalRadius - value * value)) + (double)this.yStart;
                }
                if (value > length - curveLength) {
                    double r = length - value;
                    return (double)(-sign) * (this.verticalRadius - Math.sqrt(this.verticalRadius * this.verticalRadius - r * r)) + (double)this.yEnd;
                }
                return (double)sign * ((value - curveLength) / (length - 2.0 * curveLength) * ((double)Math.abs(this.yEnd - this.yStart) - 2.0 * curveHeight) + curveHeight) + (double)this.yStart;
            }
            case CABLE: {
                if (value < 0.5) {
                    return this.yStart;
                }
                if (value > length - 0.5) {
                    return this.yEnd;
                }
                double cableOffsetValue = value - 0.5;
                double offsetLength = length - 1.0;
                double posY = (double)this.yStart + (double)(this.yEnd - this.yStart) * cableOffsetValue / offsetLength;
                double dip = offsetLength * offsetLength / 4.0 / 1000.0;
                return posY + (dip > 8.0 ? 8.0 / dip : 1.0) * (cableOffsetValue - offsetLength) * cableOffsetValue / 1000.0;
            }
        }
        double intercept = length / 2.0;
        if (value < intercept) {
            yChange = (double)(this.yEnd - this.yStart) / 2.0;
            yInitial = this.yStart;
            offsetValue = value;
        } else {
            yChange = (double)(this.yStart - this.yEnd) / 2.0;
            yInitial = this.yEnd;
            offsetValue = length - value;
        }
        return yChange * offsetValue * offsetValue / (intercept * intercept) + yInitial;
    }

    private double getVTheta() {
        double height = Math.abs(this.yEnd - this.yStart);
        double length = this.getLength();
        return 2.0 * Math.atan2(Math.sqrt(height * height - 4.0 * this.verticalRadius * height + length * length) - length, height - 4.0 * this.verticalRadius);
    }

    private static Vector getPositionXZ(double h, double k, double r, double t, double radiusOffset, boolean isStraight) {
        if (isStraight) {
            return new Vector(h * t + k * ((Math.abs(h) >= 0.5 && Math.abs(k) >= 0.5 ? 0.0 : r) + radiusOffset) + 0.5, 0.0, k * t + h * (r - radiusOffset) + 0.5);
        }
        return new Vector(h + (r + radiusOffset) * Math.cos(t / r) + 0.5, 0.0, k + (r + radiusOffset) * Math.sin(t / r) + 0.5);
    }

    private static double getTBounds(double x, double h, double z, double k, double r) {
        return Math.atan2(z - k, x - h) * r;
    }

    private static double getTBounds(double x, double h, double z, double k, double r, double tStart, boolean reverse) {
        double t = RailMath.getTBounds(x, h, z, k, r);
        if (t < tStart && !reverse) {
            return t + Math.PI * 2 * r;
        }
        if (t > tStart && reverse) {
            return t - Math.PI * 2 * r;
        }
        return t;
    }

    @FunctionalInterface
    public static interface RenderRail {
        public void renderRail(double var1, double var3, double var5, double var7, double var9, double var11, double var13, double var15, double var17, double var19);
    }
}

