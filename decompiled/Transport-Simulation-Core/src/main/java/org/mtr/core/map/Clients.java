/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.map;

import org.mtr.core.generated.map.ClientsSchema;
import org.mtr.core.map.Client;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public final class Clients
extends ClientsSchema {
    public Clients(long currentMillis, ObjectArrayList<Client> clients) {
        super(currentMillis);
        this.clients.addAll(clients);
    }
}

