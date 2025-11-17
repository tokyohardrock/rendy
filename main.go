package main

import (
	"log"
	"math"
	"time"
)

const (
	d     = 2.5 // camera Z-coordinate
	gridW = 40  // grid width
	gridH = 40  // grid height
	fps   = 60
	delta = math.Pi * 0.01
)

type vertices3D struct {
	x []float64
	y []float64
	z []float64
}

type face struct {
	v  []int
	vn int
}

type vertices2D struct {
	x []int
	y []int
}

var vrtcs vertices3D = vertices3D{
	x: []float64{-1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0},
	y: []float64{-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0},
	z: []float64{-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0},
}

var faces []face = []face{
	{v: []int{3, 7, 8}, vn: 1},
	{v: []int{3, 8, 4}, vn: 1},

	{v: []int{1, 5, 6}, vn: 2},
	{v: []int{1, 6, 2}, vn: 2},

	{v: []int{7, 3, 2}, vn: 3},
	{v: []int{7, 2, 6}, vn: 3},

	{v: []int{4, 8, 5}, vn: 4},
	{v: []int{4, 5, 1}, vn: 4},

	{v: []int{8, 7, 6}, vn: 5},
	{v: []int{8, 6, 5}, vn: 5},

	{v: []int{3, 4, 1}, vn: 6},
	{v: []int{3, 1, 2}, vn: 6},
}

var xyzRotations = []float64{0, 0, 0}

var ascii []byte = []byte{' ', '.', ':', '-', '=', '+', '*', '#', '%', '@'}

func normalizeAndCenter(vrt *vertices3D) {
	if len(vrt.x) == 0 || len(vrt.y) == 0 || len(vrt.z) == 0 {
		log.Fatal("Empty vertices — nothing to normalize")
	}
	var maxElem float64
	var cx, cy, cz float64
	for i := range vrt.x {
		val := math.Max(math.Abs(vrt.x[i]), math.Max(math.Abs(vrt.y[i]), math.Abs(vrt.z[i])))
		maxElem = math.Max(maxElem, val)
		cx += vrt.x[i]
		cy += vrt.y[i]
		cz += vrt.z[i]
	}
	if maxElem == 0 {
		log.Fatal("Singularity detected — normalization aborted")
	}
	cx /= float64(len(vrt.x))
	cy /= float64(len(vrt.y))
	cz /= float64(len(vrt.z))
	scale := 1 / maxElem
	for i := range vrt.x {
		vrt.x[i] = (vrt.x[i] - cx) * scale
		vrt.y[i] = (vrt.y[i] - cy) * scale
		vrt.z[i] = (vrt.z[i] - cz) * scale
	}
}

func getTransformedCoords(x, y, z float64) (float64, float64, float64) {
	var coordsMatrix = [][]float64{
		{x},
		{y},
		{z},
	}
	var xRot = xRotationMatrix(xyzRotations[0])
	var yRot = yRotationMatrix(xyzRotations[1])
	var zRot = zRotationMatrix(xyzRotations[2])
	var resultTransformMatrix = mulitplyMatrices(mulitplyMatrices(xRot, yRot), zRot)
	var result = mulitplyMatrices(resultTransformMatrix, coordsMatrix)
	return result[0][0], result[1][0], result[2][0]
}

func getScreenCoord(x, y, z float64) (float64, float64) {
	var x_proj = x / (z + d)
	var y_proj = y / (z + d)
	var scrnX = (x_proj + 1) * (gridW - 1) * 0.5
	var scrnY = (1 - y_proj) * (gridH - 1) * 0.5
	return scrnX, scrnY
}

func scanlineFilling(coords vertices2D) vertices2D {
	var filling vertices2D = vertices2D{
		x: make([]int, 0, 5),
		y: make([]int, 0, 5),
	}
	for i := 0; i < len(coords.y)-1; i++ {
		for j := 0; j < len(coords.y)-1-i; j++ {
			if coords.y[j] > coords.y[j+1] {
				coords.x[j], coords.x[j+1] = coords.x[j+1], coords.x[j]
				coords.y[j], coords.y[j+1] = coords.y[j+1], coords.y[j]
			}
		}
	}
	for y := coords.y[0]; y < coords.y[1]; y++ {
		if coords.y[1] == coords.y[0] || coords.y[2] == coords.y[0] {
			continue
		}
		var lim1 int = coords.x[0] + (y-coords.y[0])*(coords.x[1]-coords.x[0])/(coords.y[1]-coords.y[0])
		var lim2 int = coords.x[0] + (y-coords.y[0])*(coords.x[2]-coords.x[0])/(coords.y[2]-coords.y[0])
		for x := min(lim1, lim2); x <= max(lim1, lim2); x++ {
			filling.x = append(filling.x, x)
			filling.y = append(filling.y, y)
		}
	}
	for y := coords.y[1]; y <= coords.y[2]; y++ {
		if coords.y[2] == coords.y[1] || coords.y[2] == coords.y[0] {
			continue
		}
		var lim1 int = coords.x[1] + (y-coords.y[1])*(coords.x[2]-coords.x[1])/(coords.y[2]-coords.y[1])
		var lim2 int = coords.x[0] + (y-coords.y[0])*(coords.x[2]-coords.x[0])/(coords.y[2]-coords.y[0])
		for x := min(lim1, lim2); x <= max(lim1, lim2); x++ {
			filling.x = append(filling.x, x)
			filling.y = append(filling.y, y)
		}
	}
	return filling
}

func draw() {
	var grid = make([][]int, gridH)
	var projCoords vertices2D = vertices2D{
		make([]int, len(vrtcs.x)),
		make([]int, len(vrtcs.y)),
	}
	for i := range grid {
		grid[i] = make([]int, gridW)
	}
	for i := range vrtcs.x {
		scrnX, scrnY := getScreenCoord(getTransformedCoords(vrtcs.x[i], vrtcs.y[i], vrtcs.z[i]))
		projCoords.x[i] = int(scrnX)
		projCoords.y[i] = int(scrnY)
	}
	for i := range faces {
		var currFace vertices2D = vertices2D{
			x: make([]int, 0, len(faces[i].v)),
			y: make([]int, 0, len(faces[i].v)),
		}
		for j := range faces[i].v {
			currFace.x = append(currFace.x, projCoords.x[faces[i].v[j]-1])
			currFace.y = append(currFace.y, projCoords.y[faces[i].v[j]-1])
		}
		var filling = scanlineFilling(currFace)
		for j := range filling.x {
			grid[filling.y[j]][filling.x[j]] = 1
		}
	}
	for i := range grid {
		for j := range grid[i] {
			print(" " + string(ascii[grid[i][j]]) + " ")
		}
		print("\n")
	}
}

func main() {
	normalizeAndCenter(&vrtcs)
	for {
		time.Sleep(time.Second / fps)
		ClearScreen()
		for i := range xyzRotations {
			xyzRotations[i] = math.Mod(xyzRotations[i], math.Pi*2) + delta
		}
		draw()
	}
}
