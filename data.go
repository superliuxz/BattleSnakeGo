package main

import (
	"bytes"
	"container/heap"
	"encoding/json"
	//"log"
	"math"
	"net/http"
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

func StraightLineDistance(x, y *Point) float64 {
	return math.Sqrt(math.Pow(float64(x.X-y.X), 2) + math.Pow(float64(x.Y-y.Y), 2))
}

func AStarSearch(start, dest *Point, board *[][]uint8, buffer *bytes.Buffer) {
	visited := make(map[Point]bool)

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	head := &Item{
		value:    start,
		priority: StraightLineDistance(start, dest) + 0, // path cost at head is 0
		parent:   nil,
	}

	heap.Push(&pq, head)

	for pq.Len() > 0 {
		curr := heap.Pop(&pq).(*Item)

		if pq.Len() == 0 && curr.value != start {
			Direction(curr, buffer)
			return
		}

		if !visited[*curr.value] {
			visited[*curr.value] = true

			for _, neighbour := range Expand(curr, dest, board, &visited) {
				//log.Println(neighbour.priority)
				heap.Push(&pq, neighbour)
			}
		}
	}
}

func Expand(curr *Item, dest *Point, board *[][]uint8, visited *map[Point]bool) []*Item {
	successor := []*Item{}
	pathCost := curr.priority
	var next *Point

	// if the bottom pos is within the board && is not any snake's body && not visited
	if curr.value.Y+1 < len(*board) &&
		(*board)[curr.value.Y+1][curr.value.X] != 1 &&
		!(*visited)[Point{X: curr.value.X, Y: curr.value.Y + 1}] {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y + 1,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + StraightLineDistance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.Y-1 >= 0 &&
		(*board)[curr.value.Y-1][curr.value.X] != 1 &&
		!(*visited)[Point{X: curr.value.X, Y: curr.value.Y - 1}] {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y - 1,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + StraightLineDistance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.X+1 < len((*board)[0]) &&
		(*board)[curr.value.Y][curr.value.X+1] != 1 &&
		!(*visited)[Point{X: curr.value.X + 1, Y: curr.value.Y}] {

		next = &Point{
			X: curr.value.X + 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + StraightLineDistance(next, dest),
			parent:   curr,
		})
	}

	if curr.value.X-1 >= 0 &&
		(*board)[curr.value.Y][curr.value.X-1] != 1 &&
		!(*visited)[Point{X: curr.value.X - 1, Y: curr.value.Y}] {

		next = &Point{
			X: curr.value.X - 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &Item{
			value:    next,
			priority: pathCost + 1 + StraightLineDistance(next, dest),
			parent:   curr,
		})
	}

	return successor
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

	// log.Println("curr: ", *curr.value, "parent: ", *curr.parent.value, "direction: ", buffer.String())
}
