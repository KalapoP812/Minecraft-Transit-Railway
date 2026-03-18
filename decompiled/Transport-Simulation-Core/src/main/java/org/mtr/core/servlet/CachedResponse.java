/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonObject
 */
package org.mtr.core.servlet;

import java.util.function.Function;
import javax.annotation.Nullable;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.com.google.gson.JsonObject;

public final class CachedResponse {
    private long expiry;
    @Nullable
    private JsonObject cache;
    private final Function<Simulator, JsonObject> function;
    private final long lifespan;

    public CachedResponse(Function<Simulator, JsonObject> function, long lifespan) {
        this.function = function;
        this.lifespan = lifespan;
    }

    public JsonObject get(Simulator simulator) {
        long currentMillis = System.currentTimeMillis();
        if (this.cache == null || currentMillis > this.expiry) {
            this.cache = this.function.apply(simulator);
            this.expiry = currentMillis + this.lifespan;
        }
        return this.cache;
    }
}

