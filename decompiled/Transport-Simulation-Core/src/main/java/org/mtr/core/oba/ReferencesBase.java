/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.com.google.gson.JsonObject
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.ReferencesBaseSchema;
import org.mtr.core.oba.References;
import org.mtr.core.oba.Trip;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.com.google.gson.JsonObject;

public abstract class ReferencesBase
extends ReferencesBaseSchema {
    protected ReferencesBase(References references) {
        super(references);
    }

    protected ReferencesBase(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public final void addRoute(int routeColor) {
        if (this.isIncludeReferences()) {
            this.references.addRoute(routeColor);
        }
    }

    public final void addStop(long platformId) {
        if (this.isIncludeReferences()) {
            this.references.addStop(platformId);
        }
    }

    public final void addTrip(Trip trip) {
        if (this.isIncludeReferences()) {
            this.references.addTrip(trip);
        }
    }

    public abstract JsonObject toJson(Simulator var1);

    protected abstract boolean isIncludeReferences();
}

