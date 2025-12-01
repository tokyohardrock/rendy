package main

import (
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

const (
	d          = 2.5 // distance to camera
	gridWidth  = 40
	gridHeight = 40
	fps        = 16
	angleDelta = math.Pi * 0.01
)

var backedLight = true

var maxGoroutines = 50_000

type point struct {
	x float64
	y float64
	z float64
}

type face struct {
	v  []int
	vn int
}

var vertices []point = []point{
	{x: 1.0, y: -1.0, z: -1.0},
	{x: 1.0, y: -1.0, z: 1.0},
	{x: -1.0, y: -1.0, z: 1.0},
	{x: -1.0, y: -1.0, z: -1.0},
	{x: 1.0, y: 1.0, z: -1.0},
	{x: 1.0, y: 1.0, z: 1.0},
	{x: -1.0, y: 1.0, z: 1.0},
	{x: -1.0, y: 1.0, z: -1.0},
}

var normals []point = []point{
	{x: 0.0, y: -1.0, z: 0.0},
	{x: 0.0, y: 1.0, z: 0.0},
	{x: 1.0, y: 0.0, z: 0.0},
	{x: -1.0, y: 0.0, z: 0.0},
	{x: 0.0, y: 0.0, z: 1.0},
	{x: 0.0, y: 0.0, z: -1.0},
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

func normalizeAndCenter(vrts *[]point) {
	var vrtLen = len(*vrts)
	if vrtLen == 0 {
		log.Fatal("Empty vertices — nothing to normalize")
	}
	var maxElem float64
	var cx, cy, cz float64
	for i := range *vrts {
		var val = math.Max(math.Abs((*vrts)[i].x), math.Max(math.Abs((*vrts)[i].y), math.Abs((*vrts)[i].z)))
		maxElem = math.Max(maxElem, val)
		cx += (*vrts)[i].x
		cy += (*vrts)[i].y
		cz += (*vrts)[i].z
	}
	if maxElem == 0 {
		log.Fatal("Singularity detected — normalization aborted")
	}
	cx /= float64(vrtLen)
	cy /= float64(vrtLen)
	cz /= float64(vrtLen)
	var scale = 1 / maxElem
	for i := range *vrts {
		(*vrts)[i].x = ((*vrts)[i].x - cx) * scale
		(*vrts)[i].y = ((*vrts)[i].y - cy) * scale
		(*vrts)[i].z = ((*vrts)[i].z - cz) * scale
	}
}

func getTransformedCoords(vrtx point) point {
	var vertexMatrix = [][]float64{
		{vrtx.x},
		{vrtx.y},
		{vrtx.z},
	}
	var xRot = xRotationMatrix(xyzRotations[0])
	var yRot = yRotationMatrix(xyzRotations[1])
	var zRot = zRotationMatrix(xyzRotations[2])
	var transformMatrix = mulitplyMatrices(mulitplyMatrices(xRot, yRot), zRot)
	var result = mulitplyMatrices(transformMatrix, vertexMatrix)
	return point{x: result[0][0], y: result[1][0], z: result[2][0]}
}

func getScreenCoord(vrtx point) point {
	var x_proj = vrtx.x / (vrtx.z + d)
	var y_proj = vrtx.y / (vrtx.z + d)
	var scrnX = (x_proj + 1) * (gridWidth - 1) * 0.5
	var scrnY = (1 - y_proj) * (gridHeight - 1) * 0.5
	return point{x: scrnX, y: scrnY}
}

func findDepth(coords []point, vrtX, vrtY, det float64) float64 {
	var l1 = ((coords[1].y-coords[2].y)*(vrtX-coords[2].x) + (coords[2].x-coords[1].x)*(vrtY-coords[2].y)) / det
	var l2 = ((coords[2].y-coords[0].y)*(vrtX-coords[2].x) + (coords[0].x-coords[2].x)*(vrtY-coords[2].y)) / det
	var l3 = 1 - l1 - l2
	var inv = l1/(coords[0].z) + l2/(coords[1].z) + l3/(coords[2].z)
	return 1 / inv
}

func fillFace(coords []point) []point {
	if len(coords) < 3 {
		log.Println("Invalid face - must be at least 3 vertices")
		return []point{}
	}
	var filling []point = make([]point, 0, 5)
	if coords[0].y > coords[1].y {
		coords[0], coords[1] = coords[1], coords[0]
	}
	if coords[1].y > coords[2].y {
		coords[1], coords[2] = coords[2], coords[1]
	}
	if coords[0].y > coords[1].y {
		coords[0], coords[1] = coords[1], coords[0]
	}
	var det = (coords[1].y-coords[2].y)*(coords[0].x-coords[2].x) + (coords[2].x-coords[1].x)*(coords[0].y-coords[2].y)
	var startY = int(math.Max(0, math.Ceil(coords[0].y)))
	var midY = int(math.Min(gridHeight-1, math.Ceil(coords[1].y)))
	var endY = int(math.Min(gridHeight-1, math.Ceil(coords[2].y)))
	for y := startY; y < midY; y++ {
		var lim1 = coords[0].x + (float64(y)-coords[0].y)*(coords[1].x-coords[0].x)/(coords[1].y-coords[0].y)
		var lim2 = coords[0].x + (float64(y)-coords[0].y)*(coords[2].x-coords[0].x)/(coords[2].y-coords[0].y)
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			filling = append(filling, point{
				x: float64(x),
				y: float64(y),
				z: findDepth(coords, float64(x), float64(y), det),
			})
		}
	}
	for y := midY; y < endY; y++ {
		var lim1 = coords[1].x + (float64(y)-coords[1].y)*(coords[2].x-coords[1].x)/(coords[2].y-coords[1].y)
		var lim2 = coords[0].x + (float64(y)-coords[0].y)*(coords[2].x-coords[0].x)/(coords[2].y-coords[0].y)
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			filling = append(filling, point{
				x: float64(x),
				y: float64(y),
				z: findDepth(coords, float64(x), float64(y), det),
			})
		}
	}
	return filling
}

func cull(normal point) bool {
	return (cameraDir[0]*normal.x + cameraDir[1]*normal.y + cameraDir[2]*normal.z) <= 0
}

func getShade(normal point) int {
	var brightness = normal.x*light[0] + normal.y*light[1] + normal.z*light[2]
	brightness = ((brightness + 3) / 6) * float64(len(ascii))
	return int(brightness)
}

func draw() {
	var startTime = time.Now()
	var grid = make([][]int, gridHeight)
	var zBuff = make([][]float64, gridHeight)
	var projCoords = make([]point, len(vertices))
	var tNormals = make([]point, len(normals))
	var wg = sync.WaitGroup{}
	var mut = sync.Mutex{}
	var sem = make(chan struct{}, maxGoroutines)
	for i := range grid {
		grid[i] = make([]int, gridWidth)
		zBuff[i] = make([]float64, gridWidth)
		for j := range zBuff[i] {
			zBuff[i][j] = math.Inf(1)
		}
	}
	for i := range vertices {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			var tPoint = getTransformedCoords(vertices[i])
			projCoords[i] = getScreenCoord(tPoint)
			projCoords[i].z = tPoint.z + d
		}(i)
	}
	wg.Wait()
	for i := range normals {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			tNormals[i] = getTransformedCoords(normals[i])
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
			if cull(tNormals[nIndex]) {
				return
			}
			var brightness int
			if backedLight {
				brightness = getShade(normals[nIndex])
			} else {
				brightness = getShade(tNormals[nIndex])
			}
			var currFace []point = make([]point, 0, len(faces[i].v))
			for j := range faces[i].v {
				var index = faces[i].v[j] - 1
				currFace = append(currFace, projCoords[index])
			}
			var filling = fillFace(currFace)
			for j := range filling {
				sem <- struct{}{}
				lWg.Add(1)
				go func(j int) {
					defer lWg.Done()
					defer func() {
						<-sem
					}()
					mut.Lock()
					if zBuff[int(filling[j].y)][int(filling[j].x)] > filling[j].z {
						grid[int(filling[j].y)][int(filling[j].x)] = brightness
						zBuff[int(filling[j].y)][int(filling[j].x)] = filling[j].z
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
	normalizeAndCenter(&vertices)
	for {
		time.Sleep(time.Second / fps)
		clearScreen()
		for i := range xyzRotations {
			xyzRotations[i] = math.Mod(xyzRotations[i], math.Pi*2) + angleDelta
		}
		draw()
	}
}
