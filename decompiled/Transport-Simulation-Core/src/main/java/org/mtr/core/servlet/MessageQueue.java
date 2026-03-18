/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.servlet;

import java.util.concurrent.LinkedBlockingDeque;
import java.util.function.Consumer;
import org.mtr.core.Main;

public final class MessageQueue<T> {
    private final LinkedBlockingDeque<T> linkedBlockingDeque = new LinkedBlockingDeque();

    public void put(T object) {
        try {
            this.linkedBlockingDeque.put(object);
        }
        catch (InterruptedException e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    public void process(Consumer<T> callback) {
        T object;
        while ((object = this.linkedBlockingDeque.poll()) != null) {
            callback.accept(object);
        }
    }
}

