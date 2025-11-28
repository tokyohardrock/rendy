package main

import (
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

const (
	d          = 2.5 // camera Z-coordinate
	gridWidth  = 40  // grid width
	gridHeight = 40  // grid height
	fps        = 16
	delta      = math.Pi * 0.01
)

var backedLight = true

var maxGoroutines = 50_000

type points struct {
	x []float64
	y []float64
	z []float64
}

type face struct {
	v  []int
	vn int
}

var vrtcs points = points{
	x: []float64{1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0},
	y: []float64{-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0},
	z: []float64{-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0},
}

var normals points = points{
	x: []float64{0.0, 0.0, 1.0, -1.0, 0.0, 0.0},
	y: []float64{-1.0, 1.0, 0.0, 0.0, 0.0, 0.0},
	z: []float64{0.0, 0.0, 0.0, 0.0, 1.0, -1.0},
}

var faces []face = []face{
	{v: []int{1, 2, 3}, vn: 1},
	{v: []int{1, 3, 4}, vn: 1},

	{v: []int{5, 7, 6}, vn: 2},
	{v: []int{5, 8, 7}, vn: 2},

	{v: []int{1, 5, 6}, vn: 3},
	{v: []int{1, 6, 2}, vn: 3},

	{v: []int{4, 7, 8}, vn: 4},
	{v: []int{4, 3, 7}, vn: 4},

	{v: []int{2, 6, 7}, vn: 5},
	{v: []int{2, 7, 3}, vn: 5},

	{v: []int{1, 8, 5}, vn: 6},
	{v: []int{1, 4, 8}, vn: 6},
}

var xyzRotations = []float64{0, 0, 0}

var light = []float64{0, 0, 1}

var cameraDir = []float64{0, 0, -1}

var ascii = []byte{' ', '.', ':', '-', '=', '+', '*', '#', '%', '@'}

func normalizeAndCenter(vrt *points) {
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
	var scale = 1 / maxElem
	for i := range vrt.x {
		vrt.x[i] = (vrt.x[i] - cx) * scale
		vrt.y[i] = (vrt.y[i] - cy) * scale
		vrt.z[i] = (vrt.z[i] - cz) * scale
	}
}

func getTransformedCoords(x, y, z float64) (float64, float64, float64) {
	var vertexMatrix = [][]float64{
		{x},
		{y},
		{z},
	}
	var xRot = xRotationMatrix(xyzRotations[0])
	var yRot = yRotationMatrix(xyzRotations[1])
	var zRot = zRotationMatrix(xyzRotations[2])
	var transformMatrix = mulitplyMatrices(mulitplyMatrices(xRot, yRot), zRot)
	var result = mulitplyMatrices(transformMatrix, vertexMatrix)
	return result[0][0], result[1][0], result[2][0]
}

func getScreenCoord(x, y, z float64) (float64, float64) {
	var x_proj = x / (z + d)
	var y_proj = y / (z + d)
	var scrnX = (x_proj + 1) * (gridWidth - 1) * 0.5
	var scrnY = (1 - y_proj) * (gridHeight - 1) * 0.5
	return scrnX, scrnY
}

func findDepth(coords points, vrtX, vrtY, det float64) float64 {
	var l1 = ((coords.y[1]-coords.y[2])*(vrtX-coords.x[2]) + (coords.x[2]-coords.x[1])*(vrtY-coords.y[2])) / det
	var l2 = ((coords.y[2]-coords.y[0])*(vrtX-coords.x[2]) + (coords.x[0]-coords.x[2])*(vrtY-coords.y[2])) / det
	var l3 = 1 - l1 - l2
	var inv = l1/(coords.z[0]) + l2/(coords.z[1]) + l3/(coords.z[2])
	return 1 / inv
}

func fillFace(coords points) points {
	if len(coords.x) < 3 {
		log.Println("Invalid face - must be at least 3 vertices")
		return points{}
	}
	var filling points = points{
		x: make([]float64, 0, 5),
		y: make([]float64, 0, 5),
		z: make([]float64, 0, 5),
	}
	var swap = func(i1, i2 int) {
		coords.x[i1], coords.x[i2] = coords.x[i2], coords.x[i1]
		coords.y[i1], coords.y[i2] = coords.y[i2], coords.y[i1]
		coords.z[i1], coords.z[i2] = coords.z[i2], coords.z[i1]
	}
	if coords.y[0] > coords.y[1] {
		swap(0, 1)
	}
	if coords.y[1] > coords.y[2] {
		swap(1, 2)
	}
	if coords.y[0] > coords.y[1] {
		swap(0, 1)
	}
	var det = (coords.y[1]-coords.y[2])*(coords.x[0]-coords.x[2]) + (coords.x[2]-coords.x[1])*(coords.y[0]-coords.y[2])
	var startY = int(math.Max(0, math.Ceil(coords.y[0])))
	var midY = int(math.Min(gridHeight-1, math.Ceil(coords.y[1])))
	var endY = int(math.Min(gridHeight-1, math.Ceil(coords.y[2])))
	for y := startY; y < midY; y++ {
		var lim1 = coords.x[0] + (float64(y)-coords.y[0])*(coords.x[1]-coords.x[0])/(coords.y[1]-coords.y[0])
		var lim2 = coords.x[0] + (float64(y)-coords.y[0])*(coords.x[2]-coords.x[0])/(coords.y[2]-coords.y[0])
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			filling.x = append(filling.x, float64(x))
			filling.y = append(filling.y, float64(y))
			filling.z = append(filling.z, findDepth(coords, float64(x), float64(y), det))
		}
	}
	for y := midY; y < endY; y++ {
		var lim1 = coords.x[1] + (float64(y)-coords.y[1])*(coords.x[2]-coords.x[1])/(coords.y[2]-coords.y[1])
		var lim2 = coords.x[0] + (float64(y)-coords.y[0])*(coords.x[2]-coords.x[0])/(coords.y[2]-coords.y[0])
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			filling.x = append(filling.x, float64(x))
			filling.y = append(filling.y, float64(y))
			filling.z = append(filling.z, findDepth(coords, float64(x), float64(y), det))
		}
	}
	return filling
}

func cull(x, y, z float64) bool {
	return (cameraDir[0]*x + cameraDir[1]*y + cameraDir[2]*z) <= 0
}

func getShade(x, y, z float64) int {
	var brightness = x*light[0] + y*light[1] + z*light[2]
	brightness = ((brightness + 3) / 6) * float64(len(ascii))
	return int(brightness) + 2
}

func draw() {
	var startTime = time.Now()
	var grid = make([][]int, gridHeight)
	var zBuff = make([][]float64, gridHeight)
	var wg = sync.WaitGroup{}
	var mut = sync.Mutex{}
	var sem = make(chan struct{}, maxGoroutines)
	var projCoords = points{
		x: make([]float64, len(vrtcs.x)),
		y: make([]float64, len(vrtcs.y)),
		z: make([]float64, len(vrtcs.z)),
	}
	var tNormals = points{
		x: make([]float64, len(normals.x)),
		y: make([]float64, len(normals.y)),
		z: make([]float64, len(normals.z)),
	}
	for i := range grid {
		grid[i] = make([]int, gridWidth)
		zBuff[i] = make([]float64, gridWidth)
		for j := range zBuff[i] {
			zBuff[i][j] = math.Inf(1)
		}
	}
	for i := range vrtcs.x {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			var tX, tY, tZ = getTransformedCoords(vrtcs.x[i], vrtcs.y[i], vrtcs.z[i])
			var scrnX, scrnY = getScreenCoord(tX, tY, tZ)
			projCoords.x[i] = scrnX
			projCoords.y[i] = scrnY
			projCoords.z[i] = tZ + d
		}(i)
	}
	wg.Wait()
	for i := range normals.x {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			tNormals.x[i], tNormals.y[i], tNormals.z[i] = getTransformedCoords(normals.x[i], normals.y[i], normals.z[i])
		}(i)
	}
	wg.Wait()
	for i := range faces {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			var nIndex = faces[i].vn - 1
			var lWg = sync.WaitGroup{}
			if cull(tNormals.x[nIndex], tNormals.y[nIndex], tNormals.z[nIndex]) {
				return
			}
			var brightness int
			if backedLight {
				brightness = getShade(normals.x[nIndex], normals.y[nIndex], normals.z[nIndex])
			} else {
				brightness = getShade(tNormals.x[nIndex], tNormals.y[nIndex], tNormals.z[nIndex])
			}
			var currFace points = points{
				x: make([]float64, 0, len(faces[i].v)),
				y: make([]float64, 0, len(faces[i].v)),
				z: make([]float64, 0, len(faces[i].v)),
			}
			for j := range faces[i].v {
				var index = faces[i].v[j] - 1
				currFace.x = append(currFace.x, projCoords.x[index])
				currFace.y = append(currFace.y, projCoords.y[index])
				currFace.z = append(currFace.z, projCoords.z[index])
			}
			var filling = fillFace(currFace)
			for j := range filling.x {
				sem <- struct{}{}
				lWg.Add(1)
				go func(j int) {
					defer lWg.Done()
					defer func() {
						<-sem
					}()
					mut.Lock()
					if zBuff[int(filling.y[j])][int(filling.x[j])] > filling.z[j] {
						grid[int(filling.y[j])][int(filling.x[j])] = brightness
						zBuff[int(filling.y[j])][int(filling.x[j])] = filling.z[j]
					}
					mut.Unlock()
				}(j)
			}
			lWg.Wait()
		}(i)
	}
	wg.Wait()
	for i := range grid {
		for j := range grid[i] {
			fmt.Print(" " + string(ascii[grid[i][j]]) + " ")
		}
		fmt.Println()
	}
	fmt.Printf("Render time: %0.1f ms", float64(time.Since(startTime))/1e6)
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
