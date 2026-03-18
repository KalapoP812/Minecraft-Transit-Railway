/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.oba;

import java.util.Collection;
import org.mtr.core.generated.oba.ListElementSchema;
import org.mtr.core.oba.Agency;
import org.mtr.core.oba.References;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class ListElement<T extends SerializedDataBase>
extends ListElementSchema {
    private final ObjectArrayList<T> list = new ObjectArrayList();
    private final boolean includeReferences;
    public static final int MAX_ENTRIES = 100;

    private ListElement(boolean includeReferences) {
        super(false, new References());
        this.includeReferences = includeReferences;
    }

    @Override
    public void serializeData(WriterBase writerBase) {
        super.serializeData(writerBase);
        writerBase.writeDataset((Collection<? extends SerializedDataBase>)this.list, "list");
    }

    @Override
    public JsonObject toJson(Simulator simulator) {
        this.references.build(simulator);
        return Utilities.getJsonObjectFromData(this);
    }

    @Override
    protected boolean isIncludeReferences() {
        return this.includeReferences;
    }

    public boolean add(T entry) {
        if (this.list.size() == 100) {
            this.limitedExceeded = true;
            return false;
        }
        this.list.add(entry);
        return true;
    }

    public static <T extends SerializedDataBase> ListElement<T> create(boolean includeReferences, Agency agency) {
        ListElement<T> listElement = new ListElement<T>(includeReferences);
        if (includeReferences) {
            listElement.references.addAgency(agency);
        }
        return listElement;
    }
}

