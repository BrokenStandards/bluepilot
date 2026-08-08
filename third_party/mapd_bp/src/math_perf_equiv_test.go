package main

import (
	"math"
	"math/rand"
	"sort"
	"testing"
)

// BluePilot: the two equivalences the hot-path rewrites in math.go depend on.
// Both replaced a correct-but-slower form measured to cost real time per tick;
// these pin the claim that the replacements are bit-identical rather than
// merely close.

// median3 must agree with sort.Float64s on EVERY input, including the NaNs
// GetCurvature emits for degenerate node triples (the dense gate cannot reject
// those: every comparison against NaN is false, so they reach the median).
func TestMedian3MatchesSortFloat64s(t *testing.T) {
	nan, inf := math.NaN(), math.Inf(1)
	pool := []float64{nan, -inf, inf, 0, -0.0, 1e-9, 0.0015, 0.008, 0.0135, 1, 2, 3}
	checked := 0
	for _, a := range pool {
		for _, b := range pool {
			for _, c := range pool {
				want := []float64{a, b, c}
				sort.Float64s(want)
				got := median3(a, b, c)
				if !sameFloat(got, want[1]) {
					t.Fatalf("median3(%v,%v,%v) = %v, sort.Float64s gives %v", a, b, c, got, want[1])
				}
				checked++
			}
		}
	}

	rng := rand.New(rand.NewSource(1))
	for i := 0; i < 200000; i++ {
		a, b, c := rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64()
		switch i % 7 { // salt in NaNs at a realistic-ish rate
		case 0:
			a = nan
		case 1:
			b = nan
		case 2:
			c = nan
		}
		want := []float64{a, b, c}
		sort.Float64s(want)
		if got := median3(a, b, c); !sameFloat(got, want[1]) {
			t.Fatalf("median3(%v,%v,%v) = %v, want %v", a, b, c, got, want[1])
		}
		checked++
	}
	t.Logf("median3 == sort.Float64s middle element over %d triples", checked)
}

func sameFloat(a, b float64) bool {
	if math.IsNaN(a) && math.IsNaN(b) {
		return true
	}
	return a == b
}

// isRampWay now tests pointer presence (HasName/HasRef) instead of reading the
// text. capnp's SetText stores a NULL pointer for "", so the two are the same
// question - but the tiles this runs on are produced by a remote generator, so
// assert it against the real shipped tile rather than trusting the writer.
func TestHasNameMatchesEmptyText(t *testing.T) {
	offline := loadRealTile(t)
	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read ways: %v", err)
	}

	oneway, ramps := 0, 0
	for i := 0; i < ways.Len(); i++ {
		w := ways.At(i)
		name, err := w.Name()
		if err != nil {
			t.Fatalf("way %d: Name(): %v", i, err)
		}
		ref, err := w.Ref()
		if err != nil {
			t.Fatalf("way %d: Ref(): %v", i, err)
		}
		if w.HasName() != (name != "") {
			t.Errorf("way %d: HasName()=%v but Name()=%q - a zero-length text pointer "+
				"would make isRampWay's fast path disagree with a text read",
				i, w.HasName(), name)
		}
		if w.HasRef() != (ref != "") {
			t.Errorf("way %d: HasRef()=%v but Ref()=%q", i, w.HasRef(), ref)
		}
		if w.OneWay() {
			oneway++
			if name == "" && ref == "" {
				ramps++
			}
		}
	}
	t.Logf("%d ways, %d oneway, %d ramp-signature; pointer presence matches text emptiness throughout",
		ways.Len(), oneway, ramps)
}

// GetTargetVelocities skips the fixed-point loop on ramp points because
// latBudget's ramp branch does not read v. Pin that: iterating must not move
// the answer by a single bit.
func TestRampVelocityNeedsNoIteration(t *testing.T) {
	for _, curvature := range []float64{1.0 / 8, 1.0 / 42, 1.0 / 69, 1.0 / 150, 1.0 / 600, 1.0 / 3000} {
		radius := 1.0 / curvature
		iterated := math.Sqrt(TARGET_LAT_ACCEL * radius)
		for i := 0; i < 4; i++ {
			iterated = math.Sqrt(latBudget(iterated, true) * radius)
		}
		got := GetTargetVelocities([]Curvature{{Curvature: curvature, IsRamp: true}})[0].Velocity
		if got != iterated {
			t.Errorf("R=%.0f m: single-shot ramp velocity %v != 4-iteration %v", radius, got, iterated)
		}
	}
}
