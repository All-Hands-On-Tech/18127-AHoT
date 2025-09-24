package org.firstinspires.ftc.teamcode.common;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

/**
 * Lightweight reflection bridge for the Panels (ftcontrol) library.
 * If the library / classes are missing, all calls no-op safely.
 */
public class PanelsPublisher {
    private Object panelsInstance;
    private Method numberMethod; // (String, double) or (String, Object)
    private Method textMethod;   // (String, String) fallback
    private Method flushMethod;  // flush/update/send
    private boolean initialized = false;
    private final Map<String, Long> rateLimitMap = new HashMap<>();

    public void init() {
        if (initialized) return;
        initialized = true;
        try {
            Class<?> panelsClass = tryClass(
                    "com.bylazar.panels.Panels",
                    "com.bylazar.panels.core.Panels"
            );
            if (panelsClass == null) return;
            Method instanceMethod = tryMethod(panelsClass, "getInstance");
            if (instanceMethod == null) instanceMethod = tryMethod(panelsClass, "get");
            if (instanceMethod == null) instanceMethod = tryMethod(panelsClass, "instance");
            if (instanceMethod == null) return;
            panelsInstance = instanceMethod.invoke(null);
            if (panelsInstance == null) return;
            // Find number method
            numberMethod = findSig(panelsInstance.getClass(), new String[]{"put","putNumber","number","set","add"}, 2);
            textMethod   = findSig(panelsInstance.getClass(), new String[]{"put","putText","text","set"}, 2);
            flushMethod  = findSig(panelsInstance.getClass(), new String[]{"flush","update","send"}, 0);
        } catch (Exception ignored) {}
    }

    private Class<?> tryClass(String... names) {
        for (String n: names) {
            try { return Class.forName(n); } catch (ClassNotFoundException ignored) {}
        }
        return null;
    }
    private Method tryMethod(Class<?> c, String name) {
        try { return c.getMethod(name); } catch (Exception ignored) { return null; }
    }
    private Method findSig(Class<?> c, String[] names, int params) {
        for (String n: names) {
            for (Method m: c.getMethods()) {
                if (m.getName().equals(n) && m.getParameterCount()==params) return m;
            }
        }
        return null;
    }

    public void putNumber(String key, double value) {
        if (panelsInstance == null || numberMethod == null) return;
        try {
            Class<?>[] ptypes = numberMethod.getParameterTypes();
            if (ptypes.length==2 && ptypes[1]==double.class) {
                numberMethod.invoke(panelsInstance, key, value);
            } else if (ptypes.length==2) {
                numberMethod.invoke(panelsInstance, key, Double.valueOf(value));
            }
        } catch (Exception ignored) {}
    }
    public void putText(String key, String value) {
        if (panelsInstance == null) return;
        try {
            Method m = textMethod;
            if (m == null) { // fallback use numberMethod with object
                numberMethod.invoke(panelsInstance, key, value);
                return;
            }
            Class<?>[] ptypes = m.getParameterTypes();
            if (ptypes.length==2) m.invoke(panelsInstance, key, value);
        } catch (Exception ignored) {}
    }
    public void flush() {
        if (panelsInstance == null || flushMethod==null) return;
        try { flushMethod.invoke(panelsInstance); } catch (Exception ignored) {}
    }

    /** Publish pose (inches, degrees) with optional rate limit (ms). No auto flush now. */
    public void publishPose(String prefix, double xIn, double yIn, double headingDeg, long minIntervalMs) {
        long now = System.currentTimeMillis();
        String gateKey = prefix+"__pose";
        Long last = rateLimitMap.get(gateKey);
        if (last!=null && (now-last)<minIntervalMs) return;
        rateLimitMap.put(gateKey, now);
        putNumber(prefix+"/x_in", xIn);
        putNumber(prefix+"/y_in", yIn);
        putNumber(prefix+"/h_deg", headingDeg);
    }

    /** Rate-limited generic number. */
    public void publishNumberRate(String key, double value, long minIntervalMs) {
        long now = System.currentTimeMillis();
        Long last = rateLimitMap.get(key);
        if (last!=null && (now-last)<minIntervalMs) return;
        rateLimitMap.put(key, now);
        putNumber(key, value);
    }

    /** Call once per loop after all puts. */
    public void flushFrame() { flush(); }
}
