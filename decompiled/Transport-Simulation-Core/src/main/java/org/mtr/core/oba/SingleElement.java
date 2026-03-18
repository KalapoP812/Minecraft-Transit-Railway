/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonObject
 */
package org.mtr.core.oba;

import javax.annotation.Nullable;
import org.mtr.core.oba.Agency;
import org.mtr.core.oba.References;
import org.mtr.core.oba.ReferencesBase;
import org.mtr.core.serializer.SerializedDataBase;
import org.mtr.core.serializer.WriterBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonObject;

public final class SingleElement<T extends SerializedDataBase>
extends ReferencesBase {
    private T entry;
    private final boolean includeReferences;

    private SingleElement(boolean includeReferences) {
        super(new References());
        this.includeReferences = includeReferences;
    }

    @Override
    public void serializeData(WriterBase writerBase) {
        super.serializeData(writerBase);
        if (this.entry != null) {
            this.entry.serializeData(writerBase.writeChild("entry"));
        }
    }

    @Override
    @Nullable
    public JsonObject toJson(Simulator simulator) {
        if (this.entry == null) {
            return null;
        }
        this.references.build(simulator);
        return Utilities.getJsonObjectFromData(this);
    }

    @Override
    protected boolean isIncludeReferences() {
        return this.includeReferences;
    }

    public void set(T entry) {
        this.entry = entry;
    }

    public static <T extends SerializedDataBase> SingleElement<T> create(boolean includeReferences, Agency agency) {
        SingleElement<T> singleElement = new SingleElement<T>(includeReferences);
        if (includeReferences) {
            singleElement.references.addAgency(agency);
        }
        return singleElement;
    }
}

