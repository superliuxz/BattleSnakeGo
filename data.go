package main

import (
	"bytes"
	"encoding/json"
	"math"
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
	// fill the cells with -1 for snakes' bodies
	for _, snk := range decoded.Snakes.Data {
		for _, pos := range snk.Body.Data {
			board[pos.Y][pos.X] = -1

		}
	}

    x := FloodFill(&decoded.You.Body.Data[0], &board, false)
    log.Println("board after 1st flood fill:", x)
	PrettyPrintBoard(&board)
	head := &(decoded.You.Body.Data[0])
	tail := &(decoded.You.Body.Data[len(decoded.You.Body.Data)-1])
	bodyLength := len(decoded.You.Body.Data)
	log.Println("body length", bodyLength)

	moves := PossibleMoves(head, &board)
	log.Println("possible moves:", *moves)

	explored := make(map[string]int)

	//boardCopy := CopyBoard(&board) // make a copy of current game board

    for len(*moves) != 0 {
		path := findPath(head, &decoded.Food.Data[0], &board)
		// if the food is not reachable, chase tail
		// TODO chase tail is not reliable. tail can be unreachable. should chase the 1st block that opens up
		if len(*path) == 0 {
			log.Println("unreachable food")
			if board[tail.Y][tail.X] == -1 {
				AssignTailWeight(tail, board)
			}
			log.Printf("head (%d, %d) starts chasing tail (%d, %d)", head.X, head.Y, tail.X, tail.Y)
			path = findPath(head, tail, &board)
		// if somehow the calculated path does not start from the head -- head is not reachable from the dest
		// then we want to:
		//					if there are still possible moves, explore those
		//					if not, pick one from the explored that will least likely to kill us
		} else if (*path)[0].X != head.X || (*path)[0].Y != head.Y {
			if len(*moves) != 0 {
				// overwrite path based on next
				for k := range *moves {
					log.Println("overwritting path", head, k)
					OverwritePath(path, k, head)
					break
				}
			} else {
				log.Println("unreachable head")
				SelectDirectionWithMostSpaces(&explored, head, &decoded.Food.Data[0], buffer)
				log.Println("explored:", explored, "picked:", buffer.String())
				return &decoded, err
			}
		}
		buffer.WriteString(CalculateDirectionFromPath(path))

		nextMove := buffer.String()
		log.Println("next move:", nextMove)

		if _, exist := (*moves)[nextMove]; !exist {
			log.Panic("congrats u stepped into another dimention by ", nextMove)
		}

		newBoard, newHead := SimulateNextMove(nextMove, head, tail, &board)

		availableSpace := FloodFill(newHead, newBoard, false) + len(explored)
		log.Println("available space after next move:", availableSpace)
		log.Println("new board:")
		PrettyPrintBoard(newBoard)

		explored[nextMove] = availableSpace

		if availableSpace < bodyLength {
			delete(*moves, nextMove)
			board[newHead.Y][newHead.X] = -1
			//log.Printf("newhead (%d, %d), head(%d, %d)", newHead.X, newHead.Y, head.X, head.Y)
			FloodFill(head, &board, true) // erase all the positive weight
			log.Println("board after remove all positive weight:")
			PrettyPrintBoard(&board)
			FloodFill(head, &board, false) // recalculate the weight
			log.Println("board after re-floodfill:")
			PrettyPrintBoard(&board)
			log.Println("moves left:", *moves)
			// discard the current move
			buffer.Reset()
		} else {
			break
		}
	}

	if buffer.String() == "" {
		SelectDirectionWithMostSpaces(&explored, head, &decoded.Food.Data[0], buffer)
	}

	log.Println("explored direction:", explored)

	log.Println("pick direction ", buffer.String())

	return &decoded, err
}

func OverwritePath(path *[]*Point, dir string, head *Point) {
	(*path)[0] = head
	switch dir {
	case "up":
		(*path)[1] = &Point{
			Y: head.Y-1,
			X: head.X,
		}
	case "right":
		(*path)[1] = &Point{
			Y: head.Y,
			X: head.X+1,
		}
	case "down":
		(*path)[1] = &Point{
			Y: head.Y+1,
			X: head.X,
		}
	case "left":
		(*path)[1] = &Point{
			Y: head.Y,
			X: head.X-1,
		}
	}
}

func Distance(x, y *Point) float64 {
	return math.Abs(float64(x.X-y.X)) + math.Abs(float64(x.Y-y.Y))
}

func TieBreaking(directions []string, start, dest *Point, buffer *bytes.Buffer) {
	tieBreak := make(map[string]float64)
	dis := float64(0)
	for _, v := range directions {
		switch v {
		case "up":
			dis = Distance(&Point{
				Y: start.Y-1,
				X: start.X,
			}, dest)
		case "right":
			dis = Distance(&Point{
				Y: start.Y,
				X: start.X+1,
			}, dest)
		case "down":
			dis = Distance(&Point{
				Y: start.Y+1,
				X: start.X,
			}, dest)
		case "left":
			dis = Distance(&Point{
				Y: start.Y,
				X: start.X-1,
			}, dest)
		}
		tieBreak[v] = dis
	}

	minDis := math.MaxFloat64
	for _, v := range tieBreak {
		if v < minDis {
			minDis = v
		}
	}
	log.Println("tie breaking:", tieBreak)
	for k, v := range tieBreak {
		if v == minDis {
			log.Println("write", k)
			buffer.WriteString(k)
			break // TODO need another heuristic to break the tie if Distance fails
		}
	}
}

func SelectDirectionWithMostSpaces(explored *map[string]int, start, dest *Point, buffer *bytes.Buffer) {
	buffer.Reset()
	maxSpaces := math.MinInt32
	dirWithMostSpaces := make([]string, 0)

	for _, v := range *explored {
		if v > maxSpaces {
			maxSpaces = v
		}
	}
	//log.Printf("max spaces %d", maxSpaces)

	for k, v := range *explored {
		if maxSpaces == v {
			dirWithMostSpaces = append(dirWithMostSpaces, k)
		}
	}

	if len(dirWithMostSpaces) == 1 {
		buffer.WriteString(dirWithMostSpaces[0])
	} else if len(dirWithMostSpaces) > 1 {
		TieBreaking(dirWithMostSpaces, start, dest, buffer)
	}
}

func AssignTailWeight(tail *Point, board [][]int8) {
	if isValid(
		&Point{
			X: tail.X,
			Y: tail.Y+1,
		}, &board) {
		board[tail.Y][tail.X] = board[tail.Y+1][tail.X]+1
		//log.Println("assign from down")
	} else if isValid(
		&Point{
			X: tail.X,
			Y: tail.Y-1,
		}, &board) {
		board[tail.Y][tail.X] = board[tail.Y-1][tail.X]+1
		//log.Print("assign from up")
	} else if isValid(
		&Point{
			X: tail.X+1,
			Y: tail.Y,
		}, &board) {
		board[tail.Y][tail.X] = board[tail.Y][tail.X+1]+1
		//log.Println("assign from right")
	} else {
		board[tail.Y][tail.X] = board[tail.Y][tail.X-1]+1
		//log.Print("assign from left")
	}
	log.Println("board after assign tail:", tail)
	PrettyPrintBoard(&board)
}

func PossibleMoves(head *Point, board *[][]int8) (*map[string]bool) {
	possibleMoves := make(map[string]bool)

	if head.Y+1 < len(*board) && (*board)[head.Y+1][head.X] != -1 {
		possibleMoves["down"] = true
	}
	if head.Y > 0 && (*board)[head.Y-1][head.X] != -1 {
		possibleMoves["up"] = true
	}
	if head.X+1 < len((*board)[0]) && (*board)[head.Y][head.X+1] != -1 {
		possibleMoves["right"] = true
	}
	if head.X > 0 && (*board)[head.Y][head.X-1] != -1 {
		possibleMoves["left"] = true
	}

	return &possibleMoves
}

func PrettyPrintBoard(board *[][]int8) {
	for _, boardRow := range *board {
		log.Printf("%2v", boardRow)
	}
}

func FloodFill(head *Point, board *[][]int8, erase bool) int {
	// takes ~1ms for 20x20 grid
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
	numSpaces := 0
    for len(queue) != 0 {
        node = queue[0]
        queue = queue[1:]
        if !isValid(node.pos, board) || isVisited(node.pos, board, erase) {
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
        if erase {
        	//log.Printf("erasing (%d, %d)", node.pos.X, node.pos.Y)
			(*board)[node.pos.Y][node.pos.X] = 0
		} else {
			(*board)[node.pos.Y][node.pos.X] = node.weight
		}
		numSpaces += 1
    }
    return numSpaces
}

func isValid(pos *Point, board *[][]int8) bool {
    if pos.Y >= len(*board) ||  pos.Y < 0 || pos.X < 0 || pos.X >= len((*board)[0]) || (*board)[pos.Y][pos.X] == -1 {
        //log.Printf("(%d, %d) is invalid", pos.X, pos.Y)
        return false
    } else {
        return true
    }
}

func isVisited(pos *Point, board *[][]int8, erase bool) bool {
	if erase {
		if (*board)[pos.Y][pos.X] == 0 {
			return true
		}
	} else {
		if (*board)[pos.Y][pos.X] > 0 {
			//log.Printf("(%d, %d) has already been checked", pos.X, pos.Y)
			return true
		}
	}
	return false
}

func findPath(start, dest *Point, board *[][]int8) *[]*Point {
    var path []*Point
    var next *Point
    // dest unreachable
    if (*board)[dest.Y][dest.X] == 0 {
        return &path
    }
    path = append(path, dest)
    curr := dest
    for curr.X != start.X || curr.Y != start.Y {
		// fancy name: gradient descent
        if  curr.Y < len(*board) - 1 && (*board)[curr.Y+1][curr.X] - (*board)[curr.Y][curr.X] == -1 {
            next = &Point{
                X: curr.X,
                Y: curr.Y+1,
            }
        } else if curr.Y > 0 && (*board)[curr.Y-1][curr.X] - (*board)[curr.Y][curr.X] == -1 {
            next = &Point{
                X: curr.X,
                Y: curr.Y-1,
            }
        } else if curr.X < len((*board)[0]) - 1 && (*board)[curr.Y][curr.X+1] - (*board)[curr.Y][curr.X] == -1 {
            next = &Point{
                X: curr.X+1,
                Y: curr.Y,
            }
        } else if curr.X > 0 && (*board)[curr.Y][curr.X-1] - (*board)[curr.Y][curr.X] == -1 {
            next = &Point{
                X: curr.X-1,
                Y: curr.Y,
            }
        }
		// if curr is right beside the dest then next is null
        if next == nil {
			next = curr
		}
        // if next is not updated by any of the above four directions
		if next.X == curr.X && next.Y == curr.Y {
			// check if start is in any four direction
			if curr.Y < len(*board)-1 && curr.X == start.X && curr.Y+1 == start.Y {
				next = &Point{
					X: curr.X,
					Y: curr.Y + 1,
				}
			} else if curr.Y > 0 && curr.X == start.X && curr.Y-1 == start.Y {
				next = &Point{
					X: curr.X,
					Y: curr.Y - 1,
				}
			} else if curr.X < len((*board)[0])-1 && curr.X+1 == start.X && curr.Y == start.Y {
				next = &Point{
					X: curr.X + 1,
					Y: curr.Y,
				}
			} else if curr.X > 0 && curr.X-1 == start.X && curr.Y == start.Y {
				next = &Point{
					X: curr.X - 1,
					Y: curr.Y,
				}
			// if somehow the gradient discontinued, and start is not in any of the four directions
			} else {
				log.Printf("smells like teen spirit: curr (%d, %d), next (%d, %d), start (%d, %d),",
					curr.X, curr.Y, next.X, next.Y, start.X, start.Y)
				break
			}
		}
		path = append(path, next)
		curr = next
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
	log.Printf("calc path from (%d, %d) to (%d, %d)", (*path)[0].X, (*path)[0].Y, (*path)[1].X, (*path)[1].Y)
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

func CopyBoard(board *[][]int8) (*[][]int8) {
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

func SimulateNextMove(direction string, head, tail *Point, board *[][]int8) (*[][]int8, *Point) {
	newBoard := make([][]int8, len(*board))
	for y := range *board {
		newBoard[y] = make([]int8, len((*board)[0]))
	}

	for y := range newBoard {
		for x:= range newBoard[y] {
			if (*board)[y][x] == -1 {
				newBoard[y][x] = -1
			}
		}
	}

	var newHead *Point
	switch direction {
		case "up":
			newBoard[head.Y-1][head.X] = -1
			newHead = &Point{
				Y: head.Y-1,
				X: head.X,
			}
		case "right":
			newBoard[head.Y][head.X+1] = -1
			newHead = &Point{
				Y: head.Y,
				X: head.X+1,
			}
		case "down":
			newBoard[head.Y+1][head.X] = -1
			newHead = &Point{
				Y: head.Y+1,
				X: head.X,
			}
		case "left":
			newBoard[head.Y][head.X-1] = -1
			newHead = &Point{
				Y: head.Y,
				X: head.X-1,
			}
	}

	newBoard[tail.Y][tail.X] = 0

	return &newBoard, newHead
}