/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import java.util.Collections;
import org.mtr.core.generated.oba.StopWithArrivalsAndDeparturesSchema;
import org.mtr.core.oba.ArrivalAndDeparture;

public final class StopWithArrivalsAndDepartures
extends StopWithArrivalsAndDeparturesSchema {
    public StopWithArrivalsAndDepartures(String stopId) {
        super(stopId);
    }

    public void add(ArrivalAndDeparture arrivalAndDeparture) {
        this.arrivalsAndDepartures.add(arrivalAndDeparture);
    }

    public void add(String platformId) {
        this.nearbyStopIds.add(platformId);
    }

    public void sort() {
        Collections.sort(this.arrivalsAndDepartures);
    }
}

