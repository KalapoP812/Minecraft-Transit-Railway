/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.BlockRailsSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.it.unimi.dsi.fastutil.ints.IntArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.longs.LongArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class BlockRails
extends BlockRailsSchema {
    public BlockRails(ObjectArrayList<String> railIds, IntArrayList signalColors) {
        this.railIds.addAll(railIds);
        signalColors.forEach(arg_0 -> ((LongArrayList)this.signalColors).add(arg_0));
    }

    public BlockRails(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void blockRails(Simulator simulator) {
        this.railIds.forEach(railId -> {
            Rail rail = (Rail)simulator.railIdMap.get(railId);
            if (rail != null) {
                rail.blockRail(this.signalColors);
            }
        });
    }
}

