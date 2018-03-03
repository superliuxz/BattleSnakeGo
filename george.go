package main

import (
	"log"
	"net/http"
	"bytes"
	"encoding/json"
	"math"
)

func NewGeorgeMoveRequest(req *http.Request, buffer *bytes.Buffer) (*MoveRequest, error) {
	decoded := MoveRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)

	// create a board of current game state
	board := make([][]int, decoded.Height)
	for y := range board {
		board[y] = make([]int, decoded.Width)
	}
	// fill the cells with 1 for snakes' bodies
	for _, snk := range decoded.Snakes.Data {
		for i, pos := range snk.Body.Data {
			if board[pos.Y][pos.X] != 0 {
				continue
			}
			board[pos.Y][pos.X] = - (len(snk.Body.Data)-i)
		}
	}

	var otherSnakes []Snake
	for _, snake := range decoded.Snakes.Data {
		if snake.Body.Data[0] == decoded.You.Body.Data[0] {
			continue
		}
		otherSnakes = append(otherSnakes, snake)
	}

	gameState := make([]*MoveState, 4)
	head := decoded.You.Body.Data[0]

	var dest *Point

	if decoded.You.Health < 15 || len(decoded.You.Body.Data) < 2 {
		preferredFood := preferredFood(&head, &decoded.Food.Data, &board, &otherSnakes)
		if len(*preferredFood) >= 1 {
			// not using Pop in case we want to do something else
			dest = (*preferredFood)[0].value
		} else {
			dest = &decoded.Food.Data[0]
		}
	} else {
		dest = &decoded.You.Body.Data[len(decoded.You.Body.Data)-1]
	}

	left := &Point{head.X - 1, head.Y}
	right := &Point{head.X + 1, head.Y}
	down := &Point{head.X, head.Y + 1}
	up := &Point{head.X, head.Y - 1}

	gameState[LEFT] = createMoveState(left, &decoded.You, &otherSnakes, &board)

	gameState[RIGHT] = createMoveState(right, &decoded.You, &otherSnakes, &board)

	gameState[UP] = createMoveState(up, &decoded.You, &otherSnakes, &board)

	gameState[DOWN] = createMoveState(down, &decoded.You, &otherSnakes, &board)

	paths := findAllPaths(&decoded.You.Body.Data[0], dest, &gameState)

	i := 0
	preferredDirs := make([]int, 4)
	for i < 4 {
		preferredDirs[i] = -1
		i++
	}
	i = 0
	bestDir := UP
	min := math.MaxInt8
	for dir, path := range *paths {
		if path == nil {
			continue
		}
		if len(*path) < min {
			bestDir = dir
			min = len(*path)
		}
	}
	switch bestDir {
	case LEFT:
		buffer.WriteString("left")
	case RIGHT:
		buffer.WriteString("right")
	case UP:
		buffer.WriteString("up")
	case DOWN:
		buffer.WriteString("down")
	}
	log.Println("next move:", buffer.String())

	return &decoded, err
}