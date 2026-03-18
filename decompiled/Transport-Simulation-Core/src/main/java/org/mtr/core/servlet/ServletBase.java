/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  javax.annotation.Nullable
 *  org.mtr.libraries.com.google.gson.JsonElement
 *  org.mtr.libraries.com.google.gson.JsonObject
 *  org.mtr.libraries.com.google.gson.JsonParser
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList
 *  org.mtr.libraries.javax.servlet.AsyncContext
 *  org.mtr.libraries.javax.servlet.ServletOutputStream
 *  org.mtr.libraries.javax.servlet.WriteListener
 *  org.mtr.libraries.javax.servlet.http.HttpServlet
 *  org.mtr.libraries.javax.servlet.http.HttpServletRequest
 *  org.mtr.libraries.javax.servlet.http.HttpServletResponse
 */
package org.mtr.core.servlet;

import java.io.Reader;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import javax.annotation.Nullable;
import org.mtr.core.Main;
import org.mtr.core.integration.Response;
import org.mtr.core.serializer.JsonReader;
import org.mtr.core.servlet.HttpResponseStatus;
import org.mtr.core.simulation.Simulator;
import org.mtr.libraries.com.google.gson.JsonElement;
import org.mtr.libraries.com.google.gson.JsonObject;
import org.mtr.libraries.com.google.gson.JsonParser;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.Object2ObjectAVLTreeMap;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectImmutableList;
import org.mtr.libraries.javax.servlet.AsyncContext;
import org.mtr.libraries.javax.servlet.ServletOutputStream;
import org.mtr.libraries.javax.servlet.WriteListener;
import org.mtr.libraries.javax.servlet.http.HttpServlet;
import org.mtr.libraries.javax.servlet.http.HttpServletRequest;
import org.mtr.libraries.javax.servlet.http.HttpServletResponse;

public abstract class ServletBase
extends HttpServlet {
    private final ObjectImmutableList<Simulator> simulators;

    protected ServletBase(ObjectImmutableList<Simulator> simulators) {
        this.simulators = simulators;
    }

    protected void doGet(HttpServletRequest httpServletRequest, HttpServletResponse httpServletResponse) {
        this.doPost(httpServletRequest, httpServletResponse);
    }

    protected void doPost(HttpServletRequest httpServletRequest, HttpServletResponse httpServletResponse) {
        block7: {
            try {
                AsyncContext asyncContext = httpServletRequest.startAsync();
                asyncContext.setTimeout(0L);
                JsonElement jsonElement = JsonParser.parseReader((Reader)httpServletRequest.getReader());
                JsonReader jsonReader = new JsonReader((JsonElement)(jsonElement.isJsonNull() ? new JsonObject() : jsonElement));
                if (ServletBase.tryGetParameter(httpServletRequest, "dimensions").equals("all")) {
                    this.simulators.forEach(simulator -> this.run(httpServletRequest, null, null, jsonReader, (Simulator)simulator));
                    ServletBase.buildResponseObject(httpServletResponse, asyncContext, null, HttpResponseStatus.OK, new String[0]);
                    break block7;
                }
                int dimension = 0;
                try {
                    dimension = Integer.parseInt(ServletBase.tryGetParameter(httpServletRequest, "dimension"));
                }
                catch (Exception exception) {
                    // empty catch block
                }
                if (dimension < 0 || dimension >= this.simulators.size()) {
                    ServletBase.buildResponseObject(httpServletResponse, asyncContext, null, HttpResponseStatus.BAD_REQUEST, "Invalid Dimension");
                } else {
                    this.run(httpServletRequest, httpServletResponse, asyncContext, jsonReader, (Simulator)this.simulators.get(dimension));
                }
            }
            catch (Exception e) {
                Main.LOGGER.error("", (Throwable)e);
            }
        }
    }

    protected abstract void getContent(String var1, String var2, Object2ObjectAVLTreeMap<String, String> var3, JsonReader var4, Simulator var5, Consumer<JsonObject> var6);

    private void run(HttpServletRequest httpServletRequest, @Nullable HttpServletResponse httpServletResponse, @Nullable AsyncContext asyncContext, JsonReader jsonReader, Simulator simulator) {
        String data;
        String endpoint;
        String path = httpServletRequest.getPathInfo();
        if (path != null) {
            String[] pathSplit = path.substring(1).split("\\.")[0].split("/");
            endpoint = pathSplit.length > 0 ? pathSplit[0] : "";
            data = pathSplit.length > 1 ? pathSplit[1] : "";
        } else {
            endpoint = "";
            data = "";
        }
        Object2ObjectAVLTreeMap parameters = new Object2ObjectAVLTreeMap();
        httpServletRequest.getParameterMap().forEach((key, values) -> {
            if (((String[])values).length > 0) {
                parameters.put(key, values[0]);
            }
        });
        simulator.run(() -> this.getContent(endpoint, data, (Object2ObjectAVLTreeMap<String, String>)parameters, jsonReader, simulator, jsonObject -> {
            if (httpServletResponse != null && asyncContext != null) {
                ServletBase.buildResponseObject(httpServletResponse, asyncContext, jsonObject, jsonObject == null ? HttpResponseStatus.NOT_FOUND : HttpResponseStatus.OK, endpoint, data);
            }
        }));
    }

    public static void sendResponse(final HttpServletResponse httpServletResponse, final AsyncContext asyncContext, String content, String contentType, final HttpResponseStatus httpResponseStatus) {
        try {
            final ServletOutputStream servletOutputStream = httpServletResponse.getOutputStream();
            final ByteBuffer byteBuffer = ByteBuffer.wrap(content.getBytes(StandardCharsets.UTF_8));
            httpServletResponse.addHeader("Content-Type", contentType);
            httpServletResponse.addHeader("Access-Control-Allow-Origin", "*");
            if (httpResponseStatus == HttpResponseStatus.REDIRECT) {
                httpServletResponse.addHeader("Location", content);
            }
            servletOutputStream.setWriteListener(new WriteListener(){

                public void onWritePossible() {
                    try {
                        while (servletOutputStream.isReady()) {
                            if (!byteBuffer.hasRemaining()) {
                                httpServletResponse.setStatus(httpResponseStatus.code);
                                asyncContext.complete();
                                return;
                            }
                            servletOutputStream.write((int)byteBuffer.get());
                        }
                    }
                    catch (Exception e) {
                        Main.LOGGER.error("", (Throwable)e);
                    }
                }

                public void onError(Throwable throwable) {
                    asyncContext.complete();
                }
            });
        }
        catch (Exception e) {
            Main.LOGGER.error("", (Throwable)e);
        }
    }

    public static String getMimeType(String fileName) {
        String fileExtension;
        String[] fileNameSplit = fileName.split("\\.");
        switch (fileExtension = fileNameSplit.length == 0 ? "" : fileNameSplit[fileNameSplit.length - 1]) {
            case "js": {
                return "text/javascript";
            }
            case "json": {
                return "application/json";
            }
        }
        return "text/" + fileExtension;
    }

    protected static String removeLastSlash(String text) {
        if (text.isEmpty()) {
            return text;
        }
        if (text.charAt(text.length() - 1) == '/') {
            return text.substring(0, text.length() - 1);
        }
        return text;
    }

    private static void buildResponseObject(HttpServletResponse httpServletResponse, AsyncContext asyncContext, @Nullable JsonObject data, HttpResponseStatus httpResponseStatus, String ... parameters) {
        StringBuilder reasonPhrase = new StringBuilder(httpResponseStatus.description);
        String trimmedParameters = Arrays.stream(parameters).filter(parameter -> !parameter.isEmpty()).collect(Collectors.joining(", "));
        if (!trimmedParameters.isEmpty()) {
            reasonPhrase.append(" - ").append(trimmedParameters);
        }
        ServletBase.sendResponse(httpServletResponse, asyncContext, new Response(httpResponseStatus.code, reasonPhrase.toString(), data).getJson().toString(), ServletBase.getMimeType("json"), httpResponseStatus);
    }

    private static String tryGetParameter(HttpServletRequest httpServletRequest, String parameter) {
        return httpServletRequest.getParameterMap().getOrDefault(parameter, new String[]{""})[0];
    }
}

