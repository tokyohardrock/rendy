package main

import "math"

func xRotationMatrix(angle float64) [][]float64 {
	return [][]float64{
		{1, 0, 0},
		{0, math.Cos(angle), -math.Sin(angle)},
		{0, math.Sin(angle), math.Cos(angle)},
	}
}

func yRotationMatrix(angle float64) [][]float64 {
	return [][]float64{
		{math.Cos(angle), 0, math.Sin(angle)},
		{0, 1, 0},
		{-math.Sin(angle), 0, math.Cos(angle)},
	}
}

func zRotationMatrix(angle float64) [][]float64 {
	return [][]float64{
		{math.Cos(angle), -math.Sin(angle), 0},
		{math.Sin(angle), math.Cos(angle), 0},
		{0, 0, 1},
	}
}

func mulitplyMatrices(fir, sec [][]float64) [][]float64 {
	if len(fir[0]) != len(sec) {
		return nil
	}
	var matrix = make([][]float64, len(fir))
	for i := range fir {
		for j := range sec[0] {
			if matrix[i] == nil {
				matrix[i] = make([]float64, len(sec[0]))
			}
			for k := range fir[i] {
				matrix[i][j] += fir[i][k] * sec[k][j]
			}
		}
	}
	return matrix
}
