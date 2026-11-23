package org.json;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * SIMULATOR stand-in for Android's org.json.JSONObject: a small JSON object (strings, numbers,
 * booleans, null, nested objects and arrays as plain Java lists) with the methods the team uses.
 */
public class JSONObject {
    final Map<String, Object> map = new LinkedHashMap<>();

    public JSONObject() {
    }

    public JSONObject(String json) throws JSONException {
        Parser p = new Parser(json);
        Object o = p.value();
        if (!(o instanceof JSONObject)) {
            throw new JSONException("not a JSON object: " + json);
        }
        map.putAll(((JSONObject) o).map);
    }

    public JSONObject put(String name, Object value) throws JSONException {
        if (name == null) {
            throw new JSONException("null name");
        }
        map.put(name, value);
        return this;
    }

    public JSONObject put(String name, double value) throws JSONException {
        return put(name, (Object) value);
    }

    public JSONObject put(String name, int value) throws JSONException {
        return put(name, (Object) value);
    }

    public JSONObject put(String name, long value) throws JSONException {
        return put(name, (Object) value);
    }

    public JSONObject put(String name, boolean value) throws JSONException {
        return put(name, (Object) value);
    }

    public boolean has(String name) {
        return map.containsKey(name);
    }

    public Object opt(String name) {
        return map.get(name);
    }

    public Object get(String name) throws JSONException {
        if (!map.containsKey(name)) {
            throw new JSONException("No value for " + name);
        }
        return map.get(name);
    }

    public String optString(String name, String fallback) {
        Object o = map.get(name);
        return o == null ? fallback : String.valueOf(o);
    }

    public String optString(String name) {
        return optString(name, "");
    }

    public double optDouble(String name, double fallback) {
        Object o = map.get(name);
        if (o instanceof Number) return ((Number) o).doubleValue();
        try {
            return o == null ? fallback : Double.parseDouble(String.valueOf(o));
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    public double optDouble(String name) {
        return optDouble(name, Double.NaN);
    }

    public int optInt(String name, int fallback) {
        double d = optDouble(name, Double.NaN);
        return Double.isNaN(d) ? fallback : (int) d;
    }

    public int optInt(String name) {
        return optInt(name, 0);
    }

    public boolean optBoolean(String name, boolean fallback) {
        Object o = map.get(name);
        return o instanceof Boolean ? (Boolean) o : fallback;
    }

    public JSONObject optJSONObject(String name) {
        Object o = map.get(name);
        return o instanceof JSONObject ? (JSONObject) o : null;
    }

    public Iterator<String> keys() {
        return new ArrayList<>(map.keySet()).iterator();
    }

    public int length() {
        return map.size();
    }

    public Object remove(String name) {
        return map.remove(name);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("{");
        boolean first = true;
        for (Map.Entry<String, Object> e : map.entrySet()) {
            if (!first) sb.append(',');
            first = false;
            sb.append(quote(e.getKey())).append(':').append(write(e.getValue()));
        }
        return sb.append('}').toString();
    }

    static String write(Object v) {
        if (v == null) return "null";
        if (v instanceof String) return quote((String) v);
        if (v instanceof Double) {
            double d = (Double) v;
            return d == Math.rint(d) && Math.abs(d) < 1e15 ? String.valueOf((long) d) : String.valueOf(d);
        }
        if (v instanceof List) {
            StringBuilder sb = new StringBuilder("[");
            boolean first = true;
            for (Object o : (List<?>) v) {
                if (!first) sb.append(',');
                first = false;
                sb.append(write(o));
            }
            return sb.append(']').toString();
        }
        return String.valueOf(v);
    }

    static String quote(String s) {
        StringBuilder sb = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            if (c == '"' || c == '\\') sb.append('\\');
            sb.append(c);
        }
        return sb.append('"').toString();
    }

    /** Tiny recursive JSON reader. */
    static class Parser {
        final String s;
        int i = 0;

        Parser(String s) {
            this.s = s;
        }

        void ws() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
        }

        Object value() throws JSONException {
            ws();
            if (i >= s.length()) throw new JSONException("unexpected end of JSON");
            char c = s.charAt(i);
            if (c == '{') {
                i++;
                JSONObject o = new JSONObject();
                ws();
                if (s.charAt(i) == '}') {
                    i++;
                    return o;
                }
                while (true) {
                    ws();
                    Object k = value();
                    ws();
                    expect(':');
                    o.map.put(String.valueOf(k), value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                        continue;
                    }
                    expect('}');
                    return o;
                }
            }
            if (c == '[') {
                i++;
                List<Object> list = new ArrayList<>();
                ws();
                if (s.charAt(i) == ']') {
                    i++;
                    return list;
                }
                while (true) {
                    list.add(value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                        continue;
                    }
                    expect(']');
                    return list;
                }
            }
            if (c == '"') {
                StringBuilder sb = new StringBuilder();
                i++;
                while (s.charAt(i) != '"') {
                    if (s.charAt(i) == '\\') i++;
                    sb.append(s.charAt(i++));
                }
                i++;
                return sb.toString();
            }
            int start = i;
            while (i < s.length() && ",}] \t\r\n".indexOf(s.charAt(i)) < 0) i++;
            String word = s.substring(start, i);
            if (word.equals("true")) return true;
            if (word.equals("false")) return false;
            if (word.equals("null")) return null;
            try {
                return Double.parseDouble(word);
            } catch (NumberFormatException e) {
                throw new JSONException("bad JSON value: " + word);
            }
        }

        void expect(char c) throws JSONException {
            ws();
            if (i >= s.length() || s.charAt(i) != c) throw new JSONException("expected " + c + " at " + i);
            i++;
        }
    }
}
