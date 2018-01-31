package main

import (
	"bytes"
	"container/heap"
	"encoding/json"
	"math"
	"net/http"
	//"log"
)

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

func NewMoveRequest(req *http.Request, buffer *bytes.Buffer) (*MoveRequest, error) {
	decoded := MoveRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)

	// create a board of current game state
	board := make([][]uint8, decoded.Height)
	for y := range board {
		board[y] = make([]uint8, decoded.Width)
	}
	// fill the cells with 1 for snakes' bodies
	for _, snk := range decoded.Snakes.Data {
		for _, pos := range snk.Body.Data {
			board[pos.Y][pos.X] = 1
		}
	}

	//log.Println(board)

	AStarSearch(&decoded.You.Body.Data[0], &decoded.Food.Data[0], &board, buffer)
	return &decoded, err
}

func NewGameStartRequest(req *http.Request) (*GameStartRequest, error) {
	decoded := GameStartRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)
	return &decoded, err
}

//func (snake Snake) Head() Point { return snake.Body.Data[0] } // do we need this

func Distance(x, y *Point) float64 {
    return math.Abs(float64(x.X-y.X)) + math.Abs(float64(x.Y-y.Y))
}

func AStarSearch(start, dest *Point, board *[][]uint8, buffer *bytes.Buffer) {
	visited := make(map[Point]bool)

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	head := &Item{
		value:    start,
		priority: Distance(start, dest) + 0, // path cost at head is 0
		parent:   nil,
	}

	heap.Push(&pq, head)

	for pq.Len() > 0 {
		curr := heap.Pop(&pq).(*Item)
		if visited[*curr.value] {
		    continue
		}

		if *curr.value == *dest {
			Direction(curr, buffer)
			return
		}

		visited[*curr.value] = true

		for _, neighbour := range *Expand(curr, dest, board) {
			if !visited[*neighbour.value] {
				heap.Push(&pq, neighbour)
			}
		}
	}
}

func Expand(curr *Item, dest *Point, board *[][]uint8) *[]*Item {
	successor := make([]*Item, 0)
	pathCost := curr.priority
	var next *Point

	if curr.value.Y+1 < len(*board) && (*board)[curr.value.Y+1][curr.value.X] != 1 {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y + 1,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + Distance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.Y-1 >= 0 && (*board)[curr.value.Y-1][curr.value.X] != 1 {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y - 1,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + Distance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.X+1 < len((*board)[0]) && (*board)[curr.value.Y][curr.value.X+1] != 1 {

		next = &Point{
			X: curr.value.X + 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + Distance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.X-1 >= 0 && (*board)[curr.value.Y][curr.value.X-1] != 1 {

		next = &Point{
			X: curr.value.X - 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + Distance(next, dest),
			parent:   curr,
		})
	}

	return &successor
}

func Direction(curr *Item, buffer *bytes.Buffer) {
	for curr.parent.parent != nil {
		curr = curr.parent
	}

	if curr.value.X-curr.parent.value.X == -1 {
		buffer.WriteString("left")
	}

	if curr.value.X-curr.parent.value.X == 1 {
		buffer.WriteString("right")
	}

	if curr.value.Y-curr.parent.value.Y == -1 {
		buffer.WriteString("up")
	}

	if curr.value.Y-curr.parent.value.Y == 1 {
		buffer.WriteString("down")
	}

	//log.Println("curr: ", *curr.value, "parent: ", *curr.parent.value, "direction: ", buffer.String())
}
