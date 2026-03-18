/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.data;

import javax.annotation.Nullable;
import org.mtr.core.data.Data;
import org.mtr.core.data.Position;
import org.mtr.core.data.SavedRailBase;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.AreaBaseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public abstract class AreaBase<T extends AreaBase<T, U>, U extends SavedRailBase<U, T>>
extends AreaBaseSchema {
    public final ObjectArraySet<U> savedRails = new ObjectArraySet();

    public AreaBase(TransportMode transportMode, Data data) {
        super(transportMode, data);
    }

    public AreaBase(ReaderBase readerBase, Data data) {
        super(readerBase, data);
        DataFixer.unpackAreaBasePositions(readerBase, (value1, value2) -> {
            this.position1 = value1;
            this.position2 = value2;
        });
    }

    @Override
    protected final Position getDefaultPosition1() {
        return new Position(0L, 0L, 0L);
    }

    @Override
    protected final Position getDefaultPosition2() {
        return new Position(0L, 0L, 0L);
    }

    @Override
    public boolean isValid() {
        return !this.name.isEmpty();
    }

    public long getMinX() {
        return Math.min(this.position1.getX(), this.position2.getX());
    }

    public long getMaxX() {
        return Math.max(this.position1.getX(), this.position2.getX());
    }

    public long getMinY() {
        return Math.min(this.position1.getY(), this.position2.getY());
    }

    public long getMaxY() {
        return Math.max(this.position1.getY(), this.position2.getY());
    }

    public long getMinZ() {
        return Math.min(this.position1.getZ(), this.position2.getZ());
    }

    public long getMaxZ() {
        return Math.max(this.position1.getZ(), this.position2.getZ());
    }

    public void setCorners(Position position1, Position position2) {
        this.position1 = position1;
        this.position2 = position2;
    }

    public boolean inArea(Position position) {
        return this.inArea(position, 0.0);
    }

    public boolean inArea(Position position, double padding) {
        return AreaBase.validCorners(this) && Utilities.isBetween(position, this.position1, this.position2, padding);
    }

    public boolean intersecting(AreaBase<T, U> areaBase) {
        return AreaBase.validCorners(this) && AreaBase.validCorners(areaBase) && (this.inThis(areaBase) || areaBase.inThis(this));
    }

    public Position getCenter() {
        return AreaBase.validCorners(this) ? new Position((this.position1.getX() + this.position2.getX()) / 2L, (this.position1.getY() + this.position2.getY()) / 2L, (this.position1.getZ() + this.position2.getZ()) / 2L) : null;
    }

    private boolean inThis(AreaBase<T, U> areaBase) {
        long x1 = areaBase.position1.getX();
        long y1 = areaBase.position1.getY();
        long z1 = areaBase.position1.getZ();
        long x2 = areaBase.position2.getX();
        long y2 = areaBase.position2.getY();
        long z2 = areaBase.position2.getZ();
        return this.inArea(areaBase.position1) || this.inArea(areaBase.position2) || this.inArea(new Position(x1, y1, z2)) || this.inArea(new Position(x1, y2, z1)) || this.inArea(new Position(x1, y2, z2)) || this.inArea(new Position(x2, y1, z1)) || this.inArea(new Position(x2, y1, z2)) || this.inArea(new Position(x2, y2, z1));
    }

    public static <T extends AreaBase<T, U>, U extends SavedRailBase<U, T>> boolean validCorners(@Nullable AreaBase<T, U> areaBase) {
        return areaBase != null && areaBase.position1 != null && areaBase.position2 != null;
    }
}

