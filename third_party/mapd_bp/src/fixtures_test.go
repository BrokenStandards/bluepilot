package main

import (
	"testing"

	capnp "capnproto.org/go/capnp/v3"
)

type testWay struct {
	name             string
	ref              string
	lanes            uint8
	oneWay           bool
	maxSpeed         float64
	maxSpeedForward  float64
	maxSpeedBackward float64
	nodes            [][2]float64 // lat, lon
}

func buildOffline(t *testing.T, ways []testWay) Offline {
	t.Helper()
	_, seg, err := capnp.NewMessage(capnp.SingleSegment(nil))
	if err != nil {
		t.Fatalf("could not create capnp message: %v", err)
	}
	offline, err := NewRootOffline(seg)
	if err != nil {
		t.Fatalf("could not create offline root: %v", err)
	}
	offline.SetMinLat(-90)
	offline.SetMinLon(-180)
	offline.SetMaxLat(90)
	offline.SetMaxLon(180)
	offline.SetOverlap(0.01)

	wayList, err := offline.NewWays(int32(len(ways)))
	if err != nil {
		t.Fatalf("could not create way list: %v", err)
	}
	for i, tw := range ways {
		w := wayList.At(i)
		if err := w.SetName(tw.name); err != nil {
			t.Fatalf("could not set way name: %v", err)
		}
		if err := w.SetRef(tw.ref); err != nil {
			t.Fatalf("could not set way ref: %v", err)
		}
		w.SetLanes(tw.lanes)
		w.SetOneWay(tw.oneWay)
		w.SetMaxSpeed(tw.maxSpeed)
		w.SetMaxSpeedForward(tw.maxSpeedForward)
		w.SetMaxSpeedBackward(tw.maxSpeedBackward)

		nodes, err := w.NewNodes(int32(len(tw.nodes)))
		if err != nil {
			t.Fatalf("could not create way nodes: %v", err)
		}
		minLat, minLon := 90.0, 180.0
		maxLat, maxLon := -90.0, -180.0
		for j, n := range tw.nodes {
			nodes.At(j).SetLatitude(n[0])
			nodes.At(j).SetLongitude(n[1])
			if n[0] < minLat {
				minLat = n[0]
			}
			if n[0] > maxLat {
				maxLat = n[0]
			}
			if n[1] < minLon {
				minLon = n[1]
			}
			if n[1] > maxLon {
				maxLon = n[1]
			}
		}
		w.SetMinLat(minLat)
		w.SetMinLon(minLon)
		w.SetMaxLat(maxLat)
		w.SetMaxLon(maxLon)
	}
	return offline
}

func wayByName(t *testing.T, offline Offline, name string) Way {
	t.Helper()
	ways, err := offline.Ways()
	if err != nil {
		t.Fatalf("could not read offline ways: %v", err)
	}
	for i := 0; i < ways.Len(); i++ {
		wName, err := ways.At(i).Name()
		if err != nil {
			continue
		}
		if wName == name {
			return ways.At(i)
		}
	}
	t.Fatalf("no way named %q in fixture", name)
	return Way{}
}
