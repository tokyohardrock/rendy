package main

import (
	"bufio"
	"log"
	"os"
	"strconv"
)

func writeVertex(coords [][]byte) {
	if len(coords) != 3 {
		log.Println("Invalid point - must be defined by 3 coordinates")
		return
	}
	x, err := strconv.ParseFloat(string(coords[0]), 64)
	if err != nil {
		log.Println(err)
	}
	y, err := strconv.ParseFloat(string(coords[1]), 64)
	if err != nil {
		log.Println(err)
	}
	z, err := strconv.ParseFloat(string(coords[2]), 64)
	if err != nil {
		log.Println(err)
	}
	vertices = append(vertices, point{
		x: x,
		y: y,
		z: z,
	})
}

func writeFace(indexes [][]byte) {
	var faceIndex = len(faces)
	faces = append(faces, face{})
	faces[faceIndex].v = make([]int, 0, len(indexes))
	for i := range indexes {
		var currNum = make([]byte, 0, 4)
		for j := range indexes[i] {
			if indexes[i][j] == '/' {
				var index, err = strconv.Atoi(string(currNum))
				if err != nil {
					log.Println(err)
				}
				faces[faceIndex].v = append(faces[faceIndex].v, index)
				break
			}
			currNum = append(currNum, indexes[i][j])
		}
	}
}

func loadData(line []byte) {
	var words [][]byte = make([][]byte, 0, 4)
	words = append(words, []byte{})
	for i := range line {
		if line[i] == '\n' {
			break
		}
		var lastIndex = len(words) - 1
		if line[i] == ' ' {
			switch {
			case len(words[lastIndex]) != 0:
				words = append(words, []byte{})
			}
			continue
		}
		words[lastIndex] = append(words[lastIndex], line[i])
	}
	switch {
	case words[0][0] == 'v' && len(words[0]) == 1:
		writeVertex(words[1:])
	case words[0][0] == 'f' && len(words[0]) == 1:
		writeFace(words[1:])
	}
}

func parseFile(path string) {
	var file, err = os.Open(path)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()
	var reader = bufio.NewReader(file)
	for {
		var line, err = reader.ReadSlice('\n')
		if err != nil {
			break
		}
		loadData(line)
	}
}
