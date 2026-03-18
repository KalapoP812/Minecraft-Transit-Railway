/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 */
package org.mtr.core.integration;

import javax.annotation.Nullable;
import org.mtr.core.generated.integration.ResponseSchema;
import org.mtr.core.tool.Utilities;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;

public final class Response
extends ResponseSchema {
    public final JsonObject data;

    public Response(int code, String text, @Nullable JsonObject data) {
        super(code, System.currentTimeMillis(), text, 1L);
        this.data = data;
    }

    public JsonObject getJson() {
        JsonObject jsonObject = Utilities.getJsonObjectFromData(this);
        if (this.data != null) {
            jsonObject.add("data", (JsonElement)this.data);
        }
        return jsonObject;
    }
}

