/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import org.mtr.core.generated.data.StationExitSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class StationExit
extends StationExitSchema
implements Comparable<StationExit> {
    public StationExit() {
    }

    public StationExit(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public String getName() {
        return this.name;
    }

    public ObjectArrayList<String> getDestinations() {
        return this.destinations;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public int compareTo(StationExit stationExit) {
        if (this.equals(stationExit)) {
            return 0;
        }
        return this.getName().compareTo(stationExit.getName());
    }
}

