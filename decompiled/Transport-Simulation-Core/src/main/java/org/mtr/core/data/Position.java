/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 */
package org.mtr.core.data;

import javax.annotation.Nullable;
import org.mtr.core.generated.data.PositionSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Vector;

public class Position
extends PositionSchema
implements Comparable<Position> {
    public Position(long x, long y, long z) {
        super(x, y, z);
    }

    public Position(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public Position(Vector railPosition) {
        this((long)Math.floor(railPosition.x()), (long)Math.floor(railPosition.y()), (long)Math.floor(railPosition.z()));
    }

    public long getX() {
        return this.x;
    }

    public long getY() {
        return this.y;
    }

    public long getZ() {
        return this.z;
    }

    public Position offset(long offsetX, long offsetY, long offsetZ) {
        return offsetX == 0L && offsetY == 0L && offsetZ == 0L ? this : new Position(this.x + offsetX, this.y + offsetY, this.z + offsetZ);
    }

    public Position offset(Position position) {
        return this.offset(position.x, position.y, position.z);
    }

    public long manhattanDistance(Position position) {
        return Math.abs(position.x - this.x) + Math.abs(position.y - this.y) + Math.abs(position.z - this.z);
    }

    @Nullable
    public static Position getMin(@Nullable Position position1, @Nullable Position position2) {
        if (position1 == null) {
            return position2;
        }
        if (position2 == null) {
            return position1;
        }
        return new Position(Math.min(position1.x, position2.x), Math.min(position1.y, position2.y), Math.min(position1.z, position2.z));
    }

    @Nullable
    public static Position getMax(@Nullable Position position1, @Nullable Position position2) {
        if (position1 == null) {
            return position2;
        }
        if (position2 == null) {
            return position1;
        }
        return new Position(Math.max(position1.x, position2.x), Math.max(position1.y, position2.y), Math.max(position1.z, position2.z));
    }

    public boolean equals(Object obj) {
        if (obj instanceof Position) {
            return this.x == ((Position)obj).x && this.y == ((Position)obj).y && this.z == ((Position)obj).z;
        }
        return super.equals(obj);
    }

    public int hashCode() {
        return (int)(((this.x & 0xFFFL) << 20) + ((this.y & 0xFFL) << 12) + (this.z & 0xFFFL));
    }

    @Override
    public int compareTo(Position position) {
        if (this.equals(position)) {
            return 0;
        }
        return this.x > position.x ? 1 : (this.x < position.x ? -1 : (this.y > position.y ? 1 : (this.y < position.y ? -1 : (this.z > position.z ? 1 : -1))));
    }
}

