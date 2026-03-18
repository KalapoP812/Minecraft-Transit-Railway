/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import java.util.UUID;
import org.mtr.core.Main;
import org.mtr.core.generated.operation.PlayerPresentResponseSchema;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public class PlayerPresentResponse
extends PlayerPresentResponseSchema {
    public PlayerPresentResponse(String playerDimension) {
        super(playerDimension);
    }

    public PlayerPresentResponse(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void verify(Simulator simulator, UUID uuid) {
        if (!this.playerDimension.equals(simulator.dimension)) {
            simulator.run(() -> {
                if (simulator.clients.removeIf(client -> client.uuid.equals(uuid))) {
                    Main.LOGGER.info("Removing player {}", uuid);
                }
            });
        }
    }
}

