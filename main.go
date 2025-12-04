package main

import (
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

const (
	d          = 1.5 // distance to camera
	gridWidth  = 100
	gridHeight = 100
	fps        = 16
	angleDelta = math.Pi * 0.01
)

var backedLight = true

var maxGoroutines = 50_000

var wg = sync.WaitGroup{}
var mut = sync.Mutex{}
var sem = make(chan struct{}, maxGoroutines)

type point struct {
	x float64
	y float64
	z float64
}

type face struct {
	v  []int
	vn point
}

var vertices = make([]point, 0, 1000)
var faces = make([]face, 0, 1000)

var xyzRotations = []float64{0, 0, 0}
var light = point{x: 0, y: 0, z: 1}
var cameraDir = point{x: 0, y: 0, z: -1}

var ascii = []byte{' ', '.', ':', '-', '=', '+', '*', '#', '%', '@'}

func normalizeAndCenter(vrts *[]point) {
	var vrtLen = float64(len(*vrts))
	if vrtLen == 0 {
		log.Fatal("Empty vertices — nothing to normalize")
	}
	var maxElem float64
	var cX, cY, cZ float64
	for i := range *vrts {
		var val = math.Max(math.Abs((*vrts)[i].x), math.Max(math.Abs((*vrts)[i].y), math.Abs((*vrts)[i].z)))
		maxElem = math.Max(maxElem, val)
		cX += (*vrts)[i].x
		cY += (*vrts)[i].y
		cZ += (*vrts)[i].z
	}
	if maxElem == 0 {
		log.Fatal("Vertices collapsed to a point — normalization aborted")
	}
	cX /= vrtLen
	cY /= vrtLen
	cZ /= vrtLen
	var scale = 1 / maxElem
	for i := range *vrts {
		(*vrts)[i].x = ((*vrts)[i].x - cX) * scale
		(*vrts)[i].y = ((*vrts)[i].y - cY) * scale
		(*vrts)[i].z = ((*vrts)[i].z - cZ) * scale
	}
}

func dot(vec1 point, vec2 point) float64 {
	return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z
}

func normalizeVec(vec point) point {
	var len = math.Sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)
	return point{vec.x / len, vec.y / len, vec.z / len}
}

func computeFaceNormal(index int) point {
	var vrt1 = faces[index].v[0] - 1
	var vrt2 = faces[index].v[1] - 1
	var vrt3 = faces[index].v[2] - 1
	var v0 = point{
		x: vertices[vrt1].x - vertices[vrt3].x,
		y: vertices[vrt1].y - vertices[vrt3].y,
		z: vertices[vrt1].z - vertices[vrt3].z,
	}
	var v1 = point{
		x: vertices[vrt2].x - vertices[vrt3].x,
		y: vertices[vrt2].y - vertices[vrt3].y,
		z: vertices[vrt2].z - vertices[vrt3].z,
	}
	var rawNormal = point{
		x: v0.y*v1.z - v0.z*v1.y,
		y: v0.z*v1.x - v0.x*v1.z,
		z: v0.x*v1.y - v0.y*v1.x,
	}
	return normalizeVec(rawNormal)
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
		var fY = float64(y)
		var lim1 = coords[0].x + (fY-coords[0].y)*(coords[1].x-coords[0].x)/(coords[1].y-coords[0].y)
		var lim2 = coords[0].x + (fY-coords[0].y)*(coords[2].x-coords[0].x)/(coords[2].y-coords[0].y)
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			var fX = float64(x)
			filling = append(filling, point{
				x: fX,
				y: fY,
				z: findDepth(coords, fX, fY, det),
			})
		}
	}
	for y := midY; y < endY; y++ {
		var fY = float64(y)
		var lim1 = coords[1].x + (fY-coords[1].y)*(coords[2].x-coords[1].x)/(coords[2].y-coords[1].y)
		var lim2 = coords[0].x + (fY-coords[0].y)*(coords[2].x-coords[0].x)/(coords[2].y-coords[0].y)
		var startX = int(math.Max(0, math.Ceil(min(lim1, lim2))))
		var endX = int(math.Min(float64(gridWidth-1), math.Floor(max(lim1, lim2))))
		for x := startX; x <= endX; x++ {
			var fX = float64(x)
			filling = append(filling, point{
				x: fX,
				y: fY,
				z: findDepth(coords, fX, fY, det),
			})
		}
	}
	return filling
}

func getShade(normal point) int {
	var brightness = dot(normal, light)
	brightness = ((brightness + 3) / 6) * float64(len(ascii))
	return int(brightness)
}

func draw() {
	var startTime = time.Now()
	var grid = make([][]int, gridHeight)
	var zBuff = make([][]float64, gridHeight)
	var projCoords = make([]point, len(vertices))
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
	for i := range faces {
		sem <- struct{}{}
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			var lWg = sync.WaitGroup{}
			var tNormal = getTransformedCoords(faces[i].vn)
			if dot(tNormal, cameraDir) <= 0 {
				return
			}
			var brightness int
			if backedLight {
				brightness = getShade(faces[i].vn)
			} else {
				brightness = getShade(tNormal)
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
	var path = "models/teapot.obj"
	parseFile(path)
	normalizeAndCenter(&vertices)
	for i := range faces {
		wg.Add(1)
		sem <- struct{}{}
		go func(i int) {
			defer wg.Done()
			defer func() {
				<-sem
			}()
			faces[i].vn = computeFaceNormal(i)
		}(i)
	}
	wg.Wait()
	for {
		time.Sleep(time.Second / fps)
		clearScreen()
		for i := range xyzRotations {
			xyzRotations[i] = math.Mod(xyzRotations[i], math.Pi*2) + angleDelta
		}
		draw()
	}
}
