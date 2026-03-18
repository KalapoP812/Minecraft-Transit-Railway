/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.javax.servlet.AsyncContext
 *  org.mtr.libraries.javax.servlet.http.HttpServlet
 *  org.mtr.libraries.javax.servlet.http.HttpServletRequest
 *  org.mtr.libraries.javax.servlet.http.HttpServletResponse
 */
package org.mtr.core.servlet;

import java.util.function.Function;
import org.mtr.core.servlet.HttpResponseStatus;
import org.mtr.core.servlet.ServletBase;
import org.mtr.libraries.javax.servlet.AsyncContext;
import org.mtr.libraries.javax.servlet.http.HttpServlet;
import org.mtr.libraries.javax.servlet.http.HttpServletRequest;
import org.mtr.libraries.javax.servlet.http.HttpServletResponse;

public abstract class WebServlet
extends HttpServlet {
    private final Function<String, String> contentProvider;
    private final String expectedPath;

    public WebServlet(Function<String, String> contentProvider, String expectedPath) {
        this.contentProvider = contentProvider;
        this.expectedPath = expectedPath;
    }

    protected final void doGet(HttpServletRequest httpServletRequest, HttpServletResponse httpServletResponse) {
        AsyncContext asyncContext = httpServletRequest.startAsync();
        asyncContext.setTimeout(0L);
        String requestUri = httpServletRequest.getRequestURI();
        if (requestUri.startsWith(this.expectedPath)) {
            String path = ServletBase.removeLastSlash(requestUri.replace(this.expectedPath, ""));
            String newPath = path.isEmpty() ? "index.html" : path;
            String content = this.contentProvider.apply(newPath);
            if (content == null) {
                ServletBase.sendResponse(httpServletResponse, asyncContext, "..", "", HttpResponseStatus.REDIRECT);
            } else {
                ServletBase.sendResponse(httpServletResponse, asyncContext, content, ServletBase.getMimeType(newPath), HttpResponseStatus.OK);
            }
        } else {
            ServletBase.sendResponse(httpServletResponse, asyncContext, "..", "", HttpResponseStatus.REDIRECT);
        }
    }
}

