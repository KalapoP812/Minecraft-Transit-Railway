/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.data.AreaBase;
import org.mtr.core.data.Data;
import org.mtr.core.data.NameColorDataBase;
import org.mtr.core.data.Platform;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.data.Siding;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.SavedRailBaseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;

public abstract class SavedRailBase<T extends SavedRailBase<T, U>, U extends AreaBase<U, T>>
extends SavedRailBaseSchema {
    public U area;

    public SavedRailBase(Position position1, Position position2, TransportMode transportMode, Data data) {
        super(position1, position2, transportMode, data);
        this.name = "1";
    }

    public SavedRailBase(ReaderBase readerBase, Data data) {
        super(DataFixer.convertSavedRailBase(readerBase), data);
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public boolean containsPos(Position position) {
        return this.position1.equals(position) || this.position2.equals(position);
    }

    public Position getMidPosition() {
        Position offsetPosition = this.position1.offset(this.position2);
        return new Position(offsetPosition.getX() / 2L, offsetPosition.getY() / 2L, offsetPosition.getZ() / 2L);
    }

    public boolean isInvalidSavedRail(Data data) {
        Rail rail = (Rail)Data.tryGet(data.positionsToRail, this.position1, this.position2);
        return rail == null || this instanceof Platform && !rail.isPlatform() || this instanceof Siding && !rail.isSiding();
    }

    public Position getRandomPosition() {
        return this.position1;
    }

    public Position getOtherPosition(Position position) {
        return position.equals(this.position1) ? this.position2 : this.position1;
    }

    public boolean closeTo(Position position, double radius) {
        return Utilities.isBetween(position, this.position1, this.position2, radius);
    }

    public double getApproximateClosestDistance(Position position, Data data) {
        Rail rail = (Rail)Data.tryGet(data.positionsToRail, this.position1, this.position2);
        if (rail == null) {
            return Double.MAX_VALUE;
        }
        double[] previousPosition = new double[]{0.0, 0.0, 0.0};
        double[] closestDistance = new double[]{Double.MAX_VALUE};
        rail.railMath.render((x1, z1, x2, z2, x3, z3, x4, z4, y1, y2) -> {
            SavedRailBase.iterateAndCheckDistance(x1, y1, z1, previousPosition, position, closestDistance);
            SavedRailBase.iterateAndCheckDistance(x3, y2, z3, previousPosition, position, closestDistance);
        }, 1.0, 0.0f, 0.0f);
        return closestDistance[0];
    }

    private static void iterateAndCheckDistance(double x, double y, double z, double[] previousPosition, Position position, double[] closestDistance) {
        if (x != previousPosition[0] || y != previousPosition[1] || z != previousPosition[2]) {
            previousPosition[0] = x;
            previousPosition[1] = y;
            previousPosition[2] = z;
            Position newPosition = new Position((long)Math.floor(x), (long)Math.floor(y), (long)Math.floor(z));
            long newDistance = newPosition.manhattanDistance(position);
            if ((double)newDistance < closestDistance[0]) {
                closestDistance[0] = newDistance;
            }
        }
    }

    private static boolean isNumber(String text) {
        try {
            Double.parseDouble(text);
            return true;
        }
        catch (Exception e) {
            return false;
        }
    }

    @Override
    public int compareTo(NameColorDataBase compare) {
        boolean thisIsNumber = SavedRailBase.isNumber(this.name);
        boolean compareIsNumber = SavedRailBase.isNumber(compare.getName());
        if (thisIsNumber && compareIsNumber) {
            int floatCompare = Float.compare(Float.parseFloat(this.name), Float.parseFloat(compare.getName()));
            return floatCompare == 0 ? super.compareTo(compare) : floatCompare;
        }
        if (thisIsNumber) {
            return -1;
        }
        if (compareIsNumber) {
            return 1;
        }
        return super.compareTo(compare);
    }
}

