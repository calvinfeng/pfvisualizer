package robot

import "math"

// MapSize sets the boundary of the map, assuming a square map.
var MapSize = 100.0

// Landmarks is a list of LandMark on the current map.
var Landmarks []Landmark

// Pose is set of three coordinates of a robot on 2D plane.
type Pose struct {
	X     float64
	Y     float64
	Theta float64
}

// Normalize resets the pose to fit into the map size.
func (p Pose) Normalize() Pose {
	if p.Theta > 2*math.Pi {
		p.Theta -= 2 * math.Pi
	} else if p.Theta < 0 {
		p.Theta += 2 * math.Pi
	}

	if p.X > MapSize {
		p.X -= MapSize
	} else if p.X < 0 {
		p.X += MapSize
	}

	if p.Y > MapSize {
		p.Y -= MapSize
	} else if p.Y < 0 {
		p.Y += MapSize
	}

	return p
}

// Landmark is a mock location on the map, notice that we are running a
// simulation
type Landmark struct {
	X float64
	Y float64
}

// Particle represents a sample of the posterior distribution of robot pose.
type Particle interface {
	SetTranslationalNoise(float64)
	SetAngularNoise(float64)
	SetMeasurementNoise(float64)
	Measurements() []float64
	Move(theta, dist float64) error
	RangeMeasure([]Landmark) error
	Weight([]Landmark) (float64, error)
	Copy() Particle
}

// ParticleFilter approximates posterior with a finite set of samples.
type ParticleFilter interface {
	SampleSize() int64
}

// Agent is a robot that moves around.
type Agent interface {
}
