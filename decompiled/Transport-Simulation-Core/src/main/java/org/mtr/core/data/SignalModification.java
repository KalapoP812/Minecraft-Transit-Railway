/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import org.mtr.core.data.Data;
import org.mtr.core.data.Position;
import org.mtr.core.data.Rail;
import org.mtr.core.generated.data.SignalModificationSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class SignalModification
extends SignalModificationSchema {
    public SignalModification(Position position1, Position position2, boolean clearAll) {
        super(position1, position2, clearAll);
    }

    public SignalModification(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    @Override
    public boolean isValid() {
        return !this.position1.equals(this.position2);
    }

    @Override
    protected Position getPosition1() {
        return this.position1;
    }

    @Override
    protected Position getPosition2() {
        return this.position2;
    }

    public void applyModificationToRail(Data data, ObjectArrayList<Rail> railsToUpdate) {
        Rail rail = (Rail)data.railIdMap.get(this.getHexId());
        if (rail != null) {
            rail.applyModification(this);
            railsToUpdate.add(rail);
        }
    }

    public void putColorToAdd(int color) {
        this.signalColorsAdd.add((long)color);
    }

    public void putColorToRemove(int color) {
        this.signalColorsRemove.add((long)color);
    }

    boolean getIsClearAll() {
        return this.clearAll;
    }

    LongArrayList getSignalColorsAdd() {
        return this.signalColorsAdd;
    }

    LongArrayList getSignalColorsRemove() {
        return this.signalColorsRemove;
    }
}

