package main

import (
	"bytes"
	"encoding/json"
	//"math"
	"net/http"
	"log"
	//"net"
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

type Node struct {
    pos *Point
    weight int8
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

    calculateWeights(&decoded.You.Body.Data[0], &board)
	PrettyPrintBoard(&board)

    path := findPath(&decoded.You.Body.Data[0], &decoded.Food.Data[0], &board)
    buffer.WriteString(CalculateDirectionFromPath(path))
    log.Println("next move:", buffer.String())
	return &decoded, err
}

func PrettyPrintBoard(board *[][]int8) {
	for _, boardRow := range *board {
		log.Printf("%2v", boardRow)
	}
}

func calculateWeights(head *Point, board *[][]int8) {
    var queue []*Node
    var node *Node
    queue = append(queue, &Node{
            pos: &Point{
                X: head.X+1,
                Y: head.Y,
            },
            weight: 1,
        })
	queue = append(queue, &Node{
		pos: &Point{
			X: head.X-1,
			Y: head.Y,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: head.X,
			Y: head.Y+1,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: head.X,
			Y: head.Y-1,
		},
		weight: 1,
        })
    for len(queue) != 0 {
        node = queue[0]
        queue = queue[1:]
        if !isValid(node.pos, board) || isVisited(node.pos, board) {
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
        (*board)[node.pos.Y][node.pos.X] = node.weight
    }
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

func findPath(start, dest *Point, board *[][]int8) *[]*Point {
    var path []*Point
    var next *Point
    // dest unreachable
    if (*board)[dest.Y][dest.X] == 0 {
        return &path
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

//func CheckBlocked(dir string, head *Point, board *[][]int8) bool {
//    var next *Point
//    switch dir {
//        case "left":
//            next = &Point{
//                X: head.X-1,
//                Y: head.Y,
//            }
//        case "right":
//            next = &Point{
//                X: head.X+1,
//                Y: head.Y,
//            }
//        case "up":
//            next = &Point{
//                X: head.X,
//                Y: head.Y-1,
//            }
//        case "down":
//            next = &Point{
//                X: head.X,
//                Y: head.Y+1,
//            }
//        default:
//            return false
//    }
//    if next.X < 0 || (*board)[next.Y][next.X-1] != 1 {
//        return false
//    }
//    return true
//}

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
