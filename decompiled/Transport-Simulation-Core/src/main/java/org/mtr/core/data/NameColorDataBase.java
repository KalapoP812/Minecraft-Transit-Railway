/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet
 */
package org.mtr.core.data;

import java.util.Locale;
import java.util.stream.Collectors;
import org.mtr.core.data.Data;
import org.mtr.core.data.TransportMode;
import org.mtr.core.generated.data.NameColorDataBaseSchema;
import org.mtr.core.generated.data.StationSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.serializer.SerializedDataBaseWithId;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArraySet;

public abstract class NameColorDataBase
extends NameColorDataBaseSchema
implements SerializedDataBaseWithId,
Comparable<NameColorDataBase> {
    private String hexId;

    public NameColorDataBase(TransportMode transportMode, Data data) {
        super(transportMode, data);
    }

    public NameColorDataBase(ReaderBase readerBase, Data data) {
        super(readerBase, data);
    }

    @Override
    public final String getHexId() {
        if (this.hexId == null) {
            this.hexId = Utilities.numberToPaddedHexString(this.id);
        }
        return this.hexId;
    }

    public final long getId() {
        return this.id;
    }

    public final String getName() {
        return this.name;
    }

    public final int getColor() {
        return (int)(this.color & 0xFFFFFFL);
    }

    public final String getColorHex() {
        return Utilities.numberToPaddedHexString(this.color, 6);
    }

    public final TransportMode getTransportMode() {
        return this.transportMode;
    }

    public final void setName(String newName) {
        this.name = newName;
    }

    public final void setColor(int newColor) {
        this.color = newColor & 0xFFFFFF;
    }

    public final boolean isTransportMode(NameColorDataBase data) {
        return this.noTransportMode() || data.noTransportMode() || data.transportMode == this.transportMode;
    }

    public final boolean isTransportMode(TransportMode transportMode) {
        return this.noTransportMode() || this.transportMode == transportMode;
    }

    private String combineNameColorId() {
        return (this.name + this.color + this.id).toLowerCase(Locale.ENGLISH);
    }

    private boolean noTransportMode() {
        return this instanceof StationSchema;
    }

    public static <T extends NameColorDataBase> ObjectArrayList<T> getDataByName(ObjectArraySet<T> dataSet, String filter) {
        return dataSet.stream().filter(data -> data.getName().toLowerCase(Locale.ENGLISH).contains(filter.toLowerCase(Locale.ENGLISH).trim())).collect(Collectors.toCollection(ObjectArrayList::new));
    }

    @Override
    public int compareTo(NameColorDataBase compare) {
        return this.combineNameColorId().compareTo(compare.combineNameColorId());
    }
}

