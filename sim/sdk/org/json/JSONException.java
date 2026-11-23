package org.json;

/** SIMULATOR stand-in for Android's org.json (checked, like the real one). */
public class JSONException extends Exception {
    public JSONException(String message) {
        super(message);
    }
}
