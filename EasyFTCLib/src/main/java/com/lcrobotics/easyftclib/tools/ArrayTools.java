package com.lcrobotics.easyftclib.tools;

import java.util.HashSet;
import java.util.Set;

public class ArrayTools {
    public static <T extends Comparable<T>> boolean containsDuplicates(T[] array) {
        Set<T> dupes = new HashSet<T>();
        for (T i : array) {
            if (!dupes.add(i)) {
                return true;
            }
        }
        return false;
    }
}
