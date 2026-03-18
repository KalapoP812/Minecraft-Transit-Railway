/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.operation;

import org.mtr.core.data.Data;
import org.mtr.core.data.Lift;
import org.mtr.core.operation.UpdateDataRequest;
import org.mtr.core.serializer.JsonReader;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class GenerateByLift {
    private final Data data;
    private final Lift lift;

    public GenerateByLift(JsonReader jsonReader, Data data) {
        this.data = data;
        this.lift = new Lift(jsonReader, data);
    }

    public void generate() {
        ObjectArrayList<Lift> liftsToModify = UpdateDataRequest.getAndRemoveMatchingLifts(this.data, this.lift);
        liftsToModify.add(this.lift);
        ((Lift)liftsToModify.get(0)).setFloors(this.lift);
        this.data.lifts.add(liftsToModify.get(0));
        this.data.sync();
    }
}

