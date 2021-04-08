package org.photonvision.common.logging;

import android.util.Log;

import java.util.function.Supplier;

public class Logger {
    private final String className;
    private final LogGroup group;

    public Logger(Class<?> clazz, LogGroup group) {
        this.className = clazz.getSimpleName();
        this.group = group;
    }

    public void trace(String o) {
        Log.d("[" + className + "]", o);
    }
    public void error(String o) {
        Log.e("[" + className + "]", o);
    }
    public void error(String o, Exception e) {
        Log.e("[" + className + "]", o);
        e.printStackTrace();
    }
    public void debug(String o) {
        Log.d("[" + className + "]", o);
    }
    public void debug(Supplier<String> o) {
        Log.d("[" + className + "]", o.get());
    }
    public void trace(Supplier<String> o) {
        Log.d("[" + className + "]", o.get());
    }
    public void warn(String s) {
        Log.w("[" + className + "]", s);
    }
    public void info(String s) {
        Log.i("[" + className + "]", s);
    }
}
