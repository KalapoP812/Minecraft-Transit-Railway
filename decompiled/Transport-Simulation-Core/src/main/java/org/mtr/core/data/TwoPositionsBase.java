/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.data;

import org.mtr.core.data.Position;
import org.mtr.core.serializer.SerializedDataBaseWithId;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public abstract class TwoPositionsBase
implements SerializedDataBaseWithId {
    private String hexId;

    @Override
    public final String getHexId() {
        if (this.hexId == null) {
            this.hexId = TwoPositionsBase.getHexId(this.getPosition1(), this.getPosition2());
        }
        return this.hexId;
    }

    public final void writePositions(ObjectArraySet<Position> positionsToUpdate) {
        positionsToUpdate.add(this.getPosition1());
        positionsToUpdate.add(this.getPosition2());
    }

    protected final boolean matchesPositions(TwoPositionsBase twoPositionsBase) {
        return this.getPosition1().equals(twoPositionsBase.getPosition1()) && this.getPosition2().equals(twoPositionsBase.getPosition2()) || this.getPosition2().equals(twoPositionsBase.getPosition1()) && this.getPosition1().equals(twoPositionsBase.getPosition2());
    }

    protected abstract Position getPosition1();

    protected abstract Position getPosition2();

    public static String getHexId(Position position1, Position position2) {
        boolean reversePositions = position1.compareTo(position2) > 0;
        return TwoPositionsBase.getHexIdRaw(reversePositions ? position2 : position1, reversePositions ? position1 : position2);
    }

    public static String getHexIdRaw(Position position1, Position position2) {
        return String.format("%s-%s-%s-%s-%s-%s", Utilities.numberToPaddedHexString(position1.getX()), Utilities.numberToPaddedHexString(position1.getY()), Utilities.numberToPaddedHexString(position1.getZ()), Utilities.numberToPaddedHexString(position2.getX()), Utilities.numberToPaddedHexString(position2.getY()), Utilities.numberToPaddedHexString(position2.getZ()));
    }
}

