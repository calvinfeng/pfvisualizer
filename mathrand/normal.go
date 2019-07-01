package mathrand

import (
	"math"
	"math/rand"
)

// Gauss draws a random number from a desired normal distribution.
func Gauss(mu, sigma float64) float64 {
	return rand.NormFloat64()*sigma + mu
}

// GaussProb computes the probability of a normal PDF at x.
func GaussProb(mu, sigma, x float64) float64 {
	return math.Exp(-1*math.Pow(mu-x, 2)/math.Pow(sigma, 2)) /
		math.Sqrt(2*math.Pi*math.Pow(sigma, 2))
}
