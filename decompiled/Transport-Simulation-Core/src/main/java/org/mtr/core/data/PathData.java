/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair
 */
package org.mtr.core.data;

import javax.annotation.Nullable;
import org.mtr.core.data.Data;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.TransportMode;
import org.mtr.core.data.TwoPositionsBase;
import org.mtr.core.generated.data.PathDataSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.ConditionalList;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntAVLTreeSet;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;

public class PathData
extends PathDataSchema
implements ConditionalList {
    @Nullable
    private Rail rail;
    public final boolean reversePositions;

    public PathData(Rail rail, long savedRailBaseId, long dwellTime, int stopIndex, Position startPosition, Position endPosition) {
        this(rail, savedRailBaseId, dwellTime, stopIndex, 0.0, 0.0, startPosition, rail.getStartAngle(startPosition), endPosition, rail.getStartAngle(endPosition));
    }

    public PathData(PathData oldPathData, double startDistance, double endDistance) {
        this(oldPathData.rail, oldPathData.savedRailBaseId, oldPathData.dwellTime, oldPathData.stopIndex, startDistance, endDistance, oldPathData.startPosition, oldPathData.startAngle, oldPathData.endPosition, oldPathData.endAngle);
        this.shape = oldPathData.shape;
        this.verticalRadius = oldPathData.verticalRadius;
        this.speedLimit = oldPathData.speedLimit;
    }

    public PathData(@Nullable Rail rail, long savedRailBaseId, long dwellTime, long stopIndex, double startDistance, double endDistance, Position startPosition, Angle startAngle, Position endPosition, Angle endAngle) {
        super(savedRailBaseId, dwellTime, stopIndex, startDistance, endDistance, startPosition, startAngle, endPosition, endAngle);
        this.rail = rail;
        boolean bl = this.reversePositions = startPosition.compareTo(endPosition) > 0;
        if (rail != null) {
            this.shape = rail.railMath.getShape();
            this.verticalRadius = rail.railMath.getVerticalRadius();
        }
    }

    public PathData(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
        this.reversePositions = this.startPosition.compareTo(this.endPosition) > 0;
    }

    @Override
    public boolean matchesCondition(double value) {
        return value >= this.startDistance;
    }

    public final Rail getRail() {
        return this.rail == null ? this.defaultRail() : this.rail;
    }

    public final long getSavedRailBaseId() {
        return this.savedRailBaseId;
    }

    public final double getStartDistance() {
        return this.startDistance;
    }

    public final double getEndDistance() {
        return this.endDistance;
    }

    public final long getDwellTime() {
        return this.dwellTime;
    }

    public final int getStopIndex() {
        return (int)this.stopIndex;
    }

    public boolean isSameRail(PathData pathData) {
        return this.startPosition.equals(pathData.startPosition) && this.endPosition.equals(pathData.endPosition);
    }

    public boolean isOppositeRail(PathData pathData) {
        return this.startPosition.equals(pathData.endPosition) && this.endPosition.equals(pathData.startPosition);
    }

    public Position getOrderedPosition1() {
        return this.reversePositions ? this.endPosition : this.startPosition;
    }

    public Position getOrderedPosition2() {
        return this.reversePositions ? this.startPosition : this.endPosition;
    }

    public Angle getFacingStart() {
        return this.getRail().getStartAngle(this.reversePositions);
    }

    public double getSpeedLimitMetersPerMillisecond() {
        return Utilities.kilometersPerHourToMetersPerMillisecond(this.getSpeedLimitKilometersPerHour());
    }

    public long getSpeedLimitKilometersPerHour() {
        return Math.max(1L, this.speedLimit);
    }

    public double getRailLength() {
        return this.rail == null ? this.endDistance - this.startDistance : this.rail.railMath.getLength();
    }

    public boolean isDescending() {
        return this.endPosition.getY() < this.startPosition.getY();
    }

    public Vector getPosition(double rawValue) {
        if (this.rail != null && this.rail.railMath.isValid()) {
            return this.rail.railMath.getPosition(rawValue, this.reversePositions);
        }
        double ratio = Utilities.clamp(rawValue / this.getRailLength(), 0.0, 1.0);
        return new Vector((double)this.startPosition.getX() + ratio * (double)(this.endPosition.getX() - this.startPosition.getX()) + 0.5, (double)this.startPosition.getY() + ratio * (double)(this.endPosition.getY() - this.startPosition.getY()), (double)this.startPosition.getZ() + ratio * (double)(this.endPosition.getZ() - this.startPosition.getZ()) + 0.5);
    }

    public String getHexId(boolean reverse) {
        return reverse ? TwoPositionsBase.getHexIdRaw(this.endPosition, this.startPosition) : TwoPositionsBase.getHexIdRaw(this.startPosition, this.endPosition);
    }

    public boolean isSignalBlocked(long vehicleId, Rail.BlockReservation blockReservation) {
        return this.getRail().isBlocked(vehicleId, blockReservation);
    }

    public IntAVLTreeSet getSignalColors() {
        return this.getRail().getSignalColors();
    }

    private void writePathCache(Data data) {
        this.rail = (Rail)Data.tryGet(data.positionsToRail, this.startPosition, this.endPosition);
        if (this.rail == null) {
            this.rail = this.defaultRail();
        } else {
            this.shape = this.rail.railMath.getShape();
            this.verticalRadius = this.rail.railMath.getVerticalRadius();
        }
    }

    private Rail defaultRail() {
        ObjectObjectImmutablePair<Angle, Angle> angles = Rail.getAngles(this.startPosition, this.startAngle.angleDegrees, this.endPosition, this.endAngle.angleDegrees);
        return Rail.newRail(this.startPosition, (Angle)(angles.left()), this.endPosition, (Angle)(angles.right()), this.shape, this.verticalRadius, (ObjectArrayList<String>)new ObjectArrayList(), this.speedLimit == 0L ? 300L : this.speedLimit, 0L, false, false, true, false, false, TransportMode.TRAIN);
    }

    public static void writePathCache(ObjectList<PathData> path, Data data, TransportMode transportMode) {
        for (int i = 0; i < path.size(); ++i) {
            PathData pathData = (PathData)path.get(i);
            pathData.writePathCache(data);
            pathData.speedLimit = PathData.getRailSpeed(path, i, transportMode.defaultSpeedKilometersPerHour);
        }
    }

    private static long getRailSpeed(ObjectList<PathData> path, int currentIndex, long defaultSpeedKilometersPerHour) {
        block0: for (int offset = 0; offset <= Math.max(currentIndex, path.size() - currentIndex - 1); ++offset) {
            PathData pathData;
            for (int sign = -1; sign <= 1 && (pathData = (PathData)Utilities.getElement(path, currentIndex + sign * offset)) != null; sign += 2) {
                if (pathData.getRail().canAccelerate()) {
                    return pathData.getRail().getSpeedLimitKilometersPerHour(pathData.reversePositions);
                }
                if (offset == 0) continue block0;
            }
        }
        return defaultSpeedKilometersPerHour;
    }
}

