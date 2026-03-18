/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList
 */
package org.mtr.core.operation;

import org.mtr.core.data.Rail;
import org.mtr.core.generated.operation.RailsResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectList;

public final class RailsResponse
extends RailsResponseSchema {
    RailsResponse() {
    }

    public RailsResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public ObjectImmutableList<Rail> getRails() {
        return new ObjectImmutableList((ObjectList)this.rails);
    }

    void add(Rail rail) {
        this.rails.add(rail);
    }
}

