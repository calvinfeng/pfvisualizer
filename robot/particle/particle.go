package particle

import (
	"errors"
	"math"

	"github.com/calvinfeng/pfvisualizer/mathrand"
	"github.com/calvinfeng/pfvisualizer/robot"
)

// New returns a particle, which is a sample of the posterior.
func New() robot.Particle {
	return &particle{}
}

type particle struct {
	tnoise       float64
	anoise       float64
	mnoise       float64
	measurements []float64
	pose         robot.Pose
}

func (p *particle) SetTranslationalNoise(n float64) {
	p.tnoise = n
}

func (p *particle) SetAngularNoise(n float64) {
	p.anoise = n
}

func (p *particle) SetMeasurementNoise(n float64) {
	p.mnoise = n
}

func (p *particle) Measurements() []float64 {
	return p.measurements
}

func (p *particle) Move(angDist, transDist float64) error {
	if transDist < 0 {
		return errors.New("robot cannot move backward")
	}

	theta := p.pose.Theta + angDist + mathrand.Gauss(0, p.anoise)
	dist := transDist + mathrand.Gauss(0, p.tnoise)

	newPose := robot.Pose{
		X:     p.pose.X + math.Cos(theta)*dist,
		Y:     p.pose.Y + math.Sin(theta)*dist,
		Theta: theta,
	}

	p.pose = newPose.Normalize()

	return nil
}

func (p *particle) RangeMeasure(marks []robot.Landmark) error {
	if len(marks) == 0 {
		return errors.New("no landmarks provided for measurement")
	}

	// We will artificially add noise to represent that our measurements are not
	// accurate.
	p.measurements = make([]float64, 0, len(marks))
	for _, mark := range marks {
		dist := math.Sqrt(math.Pow((p.pose.X-mark.X), 2) + math.Pow((p.pose.Y-mark.Y), 2))
		dist += mathrand.Gauss(0, p.mnoise)
		p.measurements = append(p.measurements, dist)
	}

	return nil
}

func (p *particle) Weight(marks []robot.Landmark) (float64, error) {
	if len(p.measurements) != len(marks) {
		return -1, errors.New("len of measurements is not equal to len of landmarks")
	}

	if len(p.measurements) == 0 {
		return -1, errors.New("no measurement done")
	}

	// Caution: if landmark coordinate is false, the following calculation will be
	// off.

	prob := 1.0
	for i, mark := range marks {
		dist := math.Sqrt(math.Pow((p.pose.X-mark.X), 2) + math.Pow((p.pose.Y-mark.Y), 2))
		prob *= mathrand.GaussProb(dist, p.mnoise, p.measurements[i])
	}

	return prob, nil
}

func (p particle) Copy() robot.Particle {
	return &p
}
