package main

import (
	"bytes"
	"encoding/json"
	//"math"
	"net/http"
	"log"
	//"net"
	"math"
)

const LEFT = 0
const DOWN = 1
const RIGHT = 2
const UP = 3

type GameStartRequest struct {
	GameId int `json:"game_id"`
	Height int `json:"height"`
	Width  int `json:"width"`
}

type GameStartResponse struct {
	Color   string  `json:"color"`
	HeadUrl *string `json:"head_url,omitempty"`
	Name    string  `json:"name"`
	Taunt   *string `json:"taunt,omitempty"`
}

type MoveRequest struct {
	Food   Food   `json:"food"`
	Height int    `json:"height"`
	Width  int    `json:"width"`
	Turn   int    `json:"turn"`
	Snakes Snakes `json:"snakes"`
	You    Snake  `json:"you"`
}

type MoveResponse struct {
	Move  string  `json:"move"`
	Taunt *string `json:"taunt,omitempty"`
}

type Point struct {
	X int `json:"x"`
	Y int `json:"y"`
}

type Food struct {
	Data []Point `json:"data"`
}

type Snakes struct {
	Data []Snake `json:"data"`
}

type Snake struct {
	Health int    `json:"health"`
	Id     string `json:"id"`
	Name   string `json:"name"`
	Taunt  string `json:"taunt"`
	Length int    `json:"length"`
	Body   Body   `json:"body"`
}

type Body struct {
	Data []Point `json:"data"`
}

type Node struct {
    pos *Point
    weight int8
}

type MoveState struct {
    BoardWeights *[][]int8
    MovesRemaining int
}

func NewMoveRequest(req *http.Request, buffer *bytes.Buffer) (*MoveRequest, error) {
	decoded := MoveRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)

	// create a board of current game state
	board := make([][]int8, decoded.Height)
	for y := range board {
		board[y] = make([]int8, decoded.Width)
	}
	// fill the cells with 1 for snakes' bodies
	for _, snk := range decoded.Snakes.Data {
		for _, pos := range snk.Body.Data {
			board[pos.Y][pos.X] = -1
		}
	}

	gameState := make([]*MoveState, 4)
    head := decoded.You.Body.Data[0]
    left := &Point{head.X - 1,head.Y}
	right := &Point{head.X + 1,head.Y}
	down := &Point{head.X,head.Y + 1}
	up := &Point{head.X,head.Y - 1}
    if isValid(left, &board) {
		gameState[LEFT] = createMoveState(left, &board)
	}
	if isValid(right, &board) {
		gameState[RIGHT] = createMoveState(right, &board)
	}
	if isValid(up, &board) {
		gameState[UP] = createMoveState(up, &board)
	}
	if isValid(down, &board) {
		gameState[DOWN] = createMoveState(down, &board)
	}

	if gameState[LEFT] != nil {
		PrettyPrintBoard(gameState[LEFT].BoardWeights)
	}
	if gameState[DOWN] != nil {
		PrettyPrintBoard(gameState[DOWN].BoardWeights)
	}
	if gameState[RIGHT] != nil {
		PrettyPrintBoard(gameState[RIGHT].BoardWeights)
	}
	if gameState[UP] != nil {
		PrettyPrintBoard(gameState[UP].BoardWeights)
	}

    paths := findAllPaths(&decoded.You.Body.Data[0], &decoded.Food.Data[0], &gameState)

	i := 0
    preferredDirs := make([]int, 4)
    for i < 4 {
		preferredDirs[i] = -1
    	i++
	}
    availableDirs := make([]int, 0)
    availableDirs = append(availableDirs, LEFT)
	availableDirs = append(availableDirs, RIGHT)
	availableDirs = append(availableDirs, DOWN)
	availableDirs = append(availableDirs, UP)
	i = 0
    for len(availableDirs) > 0 {
		min := math.MaxInt8
		bestDir := -1
		for dir, path := range *paths {
			if dir == preferredDirs[0] || dir == preferredDirs[1] || dir == preferredDirs[2] || dir == preferredDirs[3] {
				continue
			}
			if path == nil {
				log.Printf("dir: %d no path", dir)
				continue
			}
			log.Printf("dir: %d len: %d", dir, len(*path))
			if len(*path) < min {
				bestDir = dir
				min = len(*path)
			}
		}
		if bestDir == -1 {
			bestDir = availableDirs[0]
		}
		preferredDirs[i] = bestDir
		log.Printf("Setting best dir %d as %d", i, bestDir)
		i++
		for j, dir := range availableDirs {
			if dir == bestDir {
				availableDirs = append(availableDirs[:j], availableDirs[j+1:]...)
				break
			}
		}
		log.Println(availableDirs)
	}
	bestDir := calculateSafeDirection(&preferredDirs, &gameState)
	switch bestDir {
	case LEFT: buffer.WriteString("left")
	case RIGHT: buffer.WriteString("right")
	case UP: buffer.WriteString("up")
	case DOWN: buffer.WriteString("down")
	}
    log.Println("next move:", buffer.String())
	return &decoded, err
}

func PrettyPrintBoard(board *[][]int8) {
	for _, boardRow := range *board {
		log.Printf("%2v", boardRow)
	}
}

func copyBoard(board *[][]int8) (*[][]int8) {
    newBoard := make([][]int8, len(*board))
    for y := range *board {
        newBoard[y] = make([]int8, len((*board)[0]))
    }

    for y := range newBoard {
        for x:= range newBoard[y] {
            newBoard[y][x] = (*board)[y][x]
        }
    }
    return &newBoard
}

func createMoveState(start *Point, originalBoard *[][]int8) *MoveState {
    if (*originalBoard)[start.Y][start.X] == -1 {
        return nil
    }

    moveState := &MoveState {
        BoardWeights: copyBoard(originalBoard),
        MovesRemaining: 0,
    }

	(*(moveState.BoardWeights))[start.Y][start.X] = -1

    var queue []*Node
    var node *Node
    queue = append(queue, &Node{
            pos: &Point{
                X: start.X+1,
                Y: start.Y,
            },
            weight: 1,
        })
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X-1,
			Y: start.Y,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X,
			Y: start.Y+1,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X,
			Y: start.Y-1,
		},
		weight: 1,
        })
    for len(queue) != 0 {
        node = queue[0]
        queue = queue[1:]
        if !isValid(node.pos, moveState.BoardWeights) || isVisited(node.pos, moveState.BoardWeights) {
            continue
        }
        queue = append(queue, &Node{
            pos: &Point{
                X: node.pos.X+1,
                Y: node.pos.Y,
            },
            weight: node.weight+1,
        })
        queue = append(queue, &Node{
            pos: &Point{
                X: node.pos.X-1,
                Y: node.pos.Y,
            },
            weight: node.weight+1,
        })
        queue = append(queue, &Node{
            pos: &Point{
                X: node.pos.X,
                Y: node.pos.Y+1,
            },
            weight: node.weight+1,
        })
        queue = append(queue, &Node{
            pos: &Point{
                X: node.pos.X,
                Y: node.pos.Y-1,
            },
            weight: node.weight+1,
        })
        (*(moveState.BoardWeights))[node.pos.Y][node.pos.X] = node.weight
        moveState.MovesRemaining += 1
    }
    
    return moveState
}

func isValid(pos *Point, board *[][]int8) bool {
    if pos.Y >= len(*board) ||  pos.Y < 0 || pos.X < 0 || pos.X >= len((*board)[0]) || (*board)[pos.Y][pos.X] == -1 {
        //log.Printf("(%d, %d) is invalid", pos.X, pos.Y)
        return false
    } else {
        return true
    }
}

func isVisited(pos *Point, board *[][]int8) bool {
    if (*board)[pos.Y][pos.X] > 0 {
        //log.Printf("(%d, %d) has already been checked", pos.X, pos.Y)
        return true
    } else {
        return false
    }
}

func findAllPaths(head, dest *Point, gameState *[]*MoveState) *[]*[]*Point {
	paths := make([]*[]*Point, 4)
	paths[LEFT] = findPath(&Point {head.X-1, head.Y}, dest, (*gameState)[LEFT])
	paths[RIGHT] = findPath(&Point {head.X+1, head.Y}, dest, (*gameState)[RIGHT])
	paths[UP] = findPath(&Point {head.X, head.Y-1}, dest, (*gameState)[UP])
	paths[DOWN] = findPath(&Point {head.X, head.Y+1}, dest, (*gameState)[DOWN])
	return &paths
}

func findPath(start, dest *Point, moveState *MoveState) *[]*Point {
	if moveState == nil {
		return nil
	}

	board := moveState.BoardWeights

    var path []*Point
    var next *Point
    // dest unreachable
    if (*board)[dest.Y][dest.X] == 0 {
        return nil
    }
    path = append(path, dest)
    next = dest
    for next.X != start.X || next.Y != start.Y {
        if  next.Y < len(*board) - 1 && ((*board)[next.Y+1][next.X] - (*board)[next.Y][next.X] == -1 || (next.Y+1 == start.Y && next.X == start.X)) {
            next = &Point{
                X: next.X,
                Y: next.Y+1,
            }
        } else if next.Y > 0 && ((*board)[next.Y-1][next.X] - (*board)[next.Y][next.X] == -1 || (next.Y-1 == start.Y && next.X == start.X)) {
            next = &Point{
                X: next.X,
                Y: next.Y-1,
            }
        } else if next.X < len((*board)[0]) - 1 && ((*board)[next.Y][next.X+1] - (*board)[next.Y][next.X] == -1 || (next.Y == start.Y && next.X+1 == start.X)) {
            next = &Point{
                X: next.X+1,
                Y: next.Y,
            }
        } else if next.X > 0 && ((*board)[next.Y][next.X-1] - (*board)[next.Y][next.X] == -1 || (next.Y == start.Y && next.X-1 == start.X)) {
            next = &Point{
                X: next.X-1,
                Y: next.Y,
            }
        }

		path = append(path, next)
    }
    ReversePath(path)
    return &path
}

func ReversePath(array []*Point) {
	for i, j := 0, len(array)-1; i<j; i, j = i+1, j-1 {
		array[i], array[j] = array[j], array[i]
	}
}

func calculateSafeDirection(preferredDirs *[]int, gameState *[]*MoveState) int {
    return (*preferredDirs)[0]
}

func NewGameStartRequest(req *http.Request) (*GameStartRequest, error) {
	decoded := GameStartRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)
	return &decoded, err
}


func CalculateDirectionFromPath(path *[]*Point) string {
	if (*path)[1].X-(*path)[0].X == -1 {
		return "left"
	} else if (*path)[1].X-(*path)[0].X == 1 {
		return "right"
	} else if (*path)[1].Y-(*path)[0].Y == -1 {
		return "up"
	} else if (*path)[1].Y-(*path)[0].Y == 1 {
		return "down"
	} else {
	    return "up" //TODO the default should be handled better
	}
}
