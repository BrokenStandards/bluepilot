package main

// Regression test for the directory file-descriptor leak in PutParam and
// RemoveParam. Before the fix each call opened the params directory to fsync
// it and never closed it; the fd was only reclaimed by os.File's finalizer,
// i.e. by GC pressure. mapd used to generate that pressure incidentally by
// re-unmarshalling the whole tile every tick — with the tile cached (see
// tile_cache.go) it no longer does, and the leak becomes a hard EMFILE crash
// after a few thousand ticks.

import (
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

func openFDCount(t *testing.T) int {
	t.Helper()
	entries, err := os.ReadDir("/proc/self/fd")
	if err != nil {
		t.Skipf("/proc/self/fd unavailable (%v); fd-leak test is Linux-only", err)
	}
	return len(entries)
}

func TestParamWritesDoNotLeakDirectoryFDs(t *testing.T) {
	if runtime.GOOS != "linux" {
		t.Skip("fd-leak test is Linux-only")
	}
	root := t.TempDir()
	dir := filepath.Join(root, "d")
	if err := os.MkdirAll(dir, 0o775); err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(dir, "TestParam")

	// Warm up so one-off allocations are not counted, then take a baseline.
	for i := 0; i < 10; i++ {
		if err := PutParam(path, []byte("warmup")); err != nil {
			t.Fatalf("PutParam: %v", err)
		}
	}
	before := openFDCount(t)

	const iterations = 300
	for i := 0; i < iterations; i++ {
		if err := PutParam(path, []byte("value")); err != nil {
			t.Fatalf("PutParam: %v", err)
		}
		if err := RemoveParam(path); err != nil {
			t.Fatalf("RemoveParam: %v", err)
		}
	}
	after := openFDCount(t)

	// The leak was one fd per PutParam plus one per RemoveParam; 600 calls
	// would show up unmistakably. Allow a small slack for runtime-internal fds.
	if after-before > 10 {
		t.Fatalf("open fds grew from %d to %d over %d PutParam/RemoveParam pairs "+
			"(directory fd leak)", before, after, iterations)
	}
}
