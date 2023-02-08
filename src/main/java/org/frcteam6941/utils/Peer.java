package org.frcteam6941.utils;

public class Peer<K, V> {
    private final K key;
    private final V value;

    public Peer(K key, V value) {
        this.key = key;
        this.value = value;
    }

    public static Peer<Range, Double> ofRangeStrategy(Range key, double value) {
        return new Peer<>(key, value);
    }

    public V getValue() {
        return value;
    }

    public K getKey() {
        return key;
    }
}
