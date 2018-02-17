package main

import (
	"bytes"
	"encoding/json"
	//"math"
	"log"
	"net/http"
	//"net"
	"container/heap"
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
	pos    *Point
	weight int
}

type MoveState struct {
	BoardWeights   *[][]int
	MovesRemaining int
}

func NewMoveRequest(req *http.Request, buffer *bytes.Buffer) (*MoveRequest, error) {
	decoded := MoveRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)

	// create a board of current game state
	board := make([][]int, decoded.Height)
	for y := range board {
		board[y] = make([]int, decoded.Width)
	}
	// fill the cells with 1 for snakes' bodies
	for _, snk := range decoded.Snakes.Data {
		for _, pos := range snk.Body.Data {
			board[pos.Y][pos.X] = -1
		}
	}

	body := decoded.You.Body.Data
	bodyLength := len(body)

	gameState := make([]*MoveState, 4)
	head := decoded.You.Body.Data[0]

	var closestFood Point

	preferredFood := preferredFood(&head, &decoded.Food.Data, &board)
	if len(*preferredFood) > 1 {
		// not using Pop in case we want to do something else
		closestFood = (*preferredFood)[0].value.(Point)
		log.Println("Closest food is", closestFood)
	} else {
		closestFood = *(&decoded.Food.Data[0])
		log.Println("None of the foods is reachable. pick ", closestFood)
	}

	left := &Point{head.X - 1, head.Y}
	right := &Point{head.X + 1, head.Y}
	down := &Point{head.X, head.Y + 1}
	up := &Point{head.X, head.Y - 1}

	gameState[LEFT] = createMoveState(left, &body, &board)

	gameState[RIGHT] = createMoveState(right, &body, &board)

	gameState[UP] = createMoveState(up, &body, &board)

	gameState[DOWN] = createMoveState(down, &body, &board)

	//if gameState[LEFT] != nil {
	//	PrettyPrintBoard(gameState[LEFT].BoardWeights)
	//}
	//if gameState[DOWN] != nil {
	//	PrettyPrintBoard(gameState[DOWN].BoardWeights)
	//}
	//if gameState[RIGHT] != nil {
	//	PrettyPrintBoard(gameState[RIGHT].BoardWeights)
	//}
	//if gameState[UP] != nil {
	//	PrettyPrintBoard(gameState[UP].BoardWeights)
	//}

	paths := findAllPaths(&decoded.You.Body.Data[0], &closestFood, &gameState)

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
	bestDir := calculateSafeDirection(&preferredDirs, bodyLength, &gameState, &head, &body)
	switch bestDir {
	case LEFT:
		buffer.WriteString("left")
	case RIGHT:
		buffer.WriteString("right")
	case UP:
		buffer.WriteString("up")
	case DOWN:
		buffer.WriteString("down")
	default:
		log.Println("Error: best direction", bestDir)
	}
	log.Println("next move:", buffer.String())

	// test the longest path is correct
	//testBoard := copyBoard(&board)
	//(*testBoard)[head.Y][head.X] = 0
	//testState := createMoveState(&head, &body, testBoard)
	//(*testState.BoardWeights)[head.Y][head.X] = 0
	//path := longestPath(&head, &closestFood, testState.BoardWeights)
	//for _, p := range path {
	//	log.Printf("path (%d %d)", p.X, p.Y)
	//}

	// test first open space
	testBoard2 := copyBoard(&board)
	PrettyPrintBoard(testBoard2)
	activeWalls := activeWalls(&head, testBoard2)

	for w := range *activeWalls {
		log.Printf("walls: (%d, %d)", w.X, w.Y)
	}
	snakes := make([][]Point, 0)
	for _, snk := range *(&decoded.Snakes.Data) {
		snakes = append(snakes, snk.Body.Data)
	}
	sp := firstOpenSpace(&snakes, activeWalls)
	log.Printf("first open space (%d, %d)", sp.X, sp.Y)

	return &decoded, err
}

func longestPath(start, dest *Point, board *[][]int) []*Point {
	// dest must be reachable from start
	log.Println("longest path between", start, "and", dest)
	PrettyPrintBoard(board)
	path := make([]*Point, 0)
	path = append(path, start)
	visited := make(map[Point]bool)
	visited[*start] = true
	for (*start).X != (*dest).X || (*start).Y != (*dest).Y {
		neighbours := getNeighboursByDistance(start, dest, board)
		heap.Init(neighbours)
		for len(*neighbours) > 0 {
			neighbour := heap.Pop(neighbours).(*Item).value.(*Point)
			//log.Println("checking", start, "neighbour", neighbour)
			if visited[*neighbour] {
				//log.Println(neighbour, "has been visited")
				continue
			}
			if math.Abs(float64((*board)[(*neighbour).Y][(*neighbour).X]-
				(*board)[start.Y][start.X])) == float64(1) {
				//log.Println("move to", neighbour)
				start = neighbour
			}
			visited[*neighbour] = true
			break
		}
		// revert path
		if start == path[len(path)-1] {
			start = path[len(path)-2]
			path = path[0 : len(path)-1]
		} else {
			path = append(path, start)
		}
	}

	return path
}

func getNeighboursByDistance(curr, dest *Point, board *[][]int) *PriorityQueue {
	left := &Point{(*curr).X - 1, (*curr).Y}
	right := &Point{(*curr).X + 1, (*curr).Y}
	down := &Point{(*curr).X, (*curr).Y + 1}
	up := &Point{(*curr).X, (*curr).Y - 1}

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	if isValid(left, board) {
		heap.Push(&pq, &Item{value: left, priority: -StraightLineDistance(left, dest)})
	}
	if isValid(right, board) {
		heap.Push(&pq, &Item{value: right, priority: -StraightLineDistance(right, dest)})
	}
	if isValid(down, board) {
		heap.Push(&pq, &Item{value: down, priority: -StraightLineDistance(down, dest)})
	}
	if isValid(up, board) {
		heap.Push(&pq, &Item{value: up, priority: -StraightLineDistance(up, dest)})
	}

	return &pq
}

func StraightLineDistance(x, y *Point) float64 {
	return math.Sqrt(math.Pow(float64(x.X-y.X), 2) + math.Pow(float64(x.Y-y.Y), 2))
}

func firstOpenSpace(snakes *[][]Point, activeWalls *map[Point]bool) *Point {
	maxLen := math.MinInt32
	for _, snk := range *snakes {
		if len(snk) > maxLen {
			maxLen = len(snk)
		}
	}
	for i := 0; i < maxLen; i++ {
		for _, snake := range *snakes {
			if i > len(snake)-1 {
				continue
			}
			if (*activeWalls)[snake[len(snake)-1-i]] {
				return &snake[len(snake)-1-i]
			}
		}
	}
	return nil
}

func preferredFood(head *Point, foods *[]Point, board *[][]int) *PriorityQueue {
	tempBoard := copyBoard(board)
	(*tempBoard)[head.Y][head.X] = 0

	var dummyBody []Point
	newBoard := createMoveState(head, &dummyBody, tempBoard).BoardWeights
	PrettyPrintBoard(newBoard)

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	for _, food := range *foods {
		log.Println("food:", food, "path cost:", (*newBoard)[food.Y][food.X])
		if (*newBoard)[food.Y][food.X] == 0 {
			continue
		}
		heap.Push(&pq, &Item{value: food, priority: float64((*newBoard)[food.Y][food.X])})
	}

	return &pq
}

func isTailStacked(body *[]Point) bool {
	if len(*body) == 0 {
		return true
	} else if len(*body) == 1 {
		return true
	} else {
		if (*body)[len(*body)-1].X == (*body)[len(*body)-2].X && (*body)[len(*body)-1].Y == (*body)[len(*body)-2].Y {
			log.Printf("Tail stacked at (%d, %d)", (*body)[len(*body)-1].X, (*body)[len(*body)-1].Y)
			return true
		} else {
			return false
		}
	}
}

func activeWalls(head *Point, board *[][]int) *map[Point]bool {
	walls := make(map[Point]bool)
	visited := make(map[Point]bool)
	var queue []*Point
	queue = append(queue, head)
	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]
		//log.Printf("checking (%d, %d) is wall", curr.X, curr.Y)
		if visited[*curr] {
			continue
		}

		left := &Point{(*curr).X - 1, (*curr).Y}
		right := &Point{(*curr).X + 1, (*curr).Y}
		down := &Point{(*curr).X, (*curr).Y + 1}
		up := &Point{(*curr).X, (*curr).Y - 1}

		if !isOutOfBound(left, board) {
			if (*board)[left.Y][left.X] == 0 {
				queue = append(queue, left)
			} else if (*board)[left.Y][left.X] == -1 && (*board)[curr.Y][curr.X] == 0 {
				//log.Printf("(%d, %d) is wall", curr.X, curr.Y)
				walls[*left] = true
			}
		}
		if !isOutOfBound(right, board) {
			if (*board)[right.Y][right.X] == 0 {
				queue = append(queue, right)
			} else if (*board)[right.Y][right.X] == -1 && (*board)[curr.Y][curr.X] == 0 {
				//log.Printf("(%d, %d) is wall", curr.X, curr.Y)
				walls[*right] = true
			}
		}
		if !isOutOfBound(up, board) {
			if (*board)[up.Y][up.X] == 0 {
				queue = append(queue, up)
			} else if (*board)[up.Y][up.X] == -1 && (*board)[curr.Y][curr.X] == 0 {
				//log.Printf("(%d, %d) is wall", curr.X, curr.Y)
				walls[*up] = true
			}
		}
		if !isOutOfBound(down, board) {
			if (*board)[down.Y][down.X] == 0 {
				queue = append(queue, down)
			} else if (*board)[down.Y][down.X] == -1 && (*board)[curr.Y][curr.X] == 0 {
				//log.Printf("(%d, %d) is wall", curr.X, curr.Y)
				walls[*down] = true
			}
		}

		visited[*curr] = true

	}
	return &walls
}

func PrettyPrintBoard(board *[][]int) {
	for _, boardRow := range *board {
		log.Printf("%2v", boardRow)
	}
}

func copyBoard(board *[][]int) *[][]int {
	newBoard := make([][]int, len(*board))
	for y := range *board {
		newBoard[y] = make([]int, len((*board)[0]))
	}

	for y := range newBoard {
		for x := range newBoard[y] {
			if (*board)[y][x] == -1 {
				newBoard[y][x] = (*board)[y][x]
			}
		}
	}
	return &newBoard
}

func createMoveState(start *Point, body *[]Point, originalBoard *[][]int) *MoveState {
	if !isValid(start, originalBoard) {
		return nil
	}

	moveState := &MoveState{
		BoardWeights:   copyBoard(originalBoard),
		MovesRemaining: 0,
	}

	(*(moveState.BoardWeights))[start.Y][start.X] = -1
	// TODO fix tail
	if !isTailStacked(body) {
		(*(moveState.BoardWeights))[(*body)[len(*body)-1].Y][(*body)[len(*body)-1].X] = 0
	}
	var queue []*Node
	var node *Node
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X + 1,
			Y: start.Y,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X - 1,
			Y: start.Y,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X,
			Y: start.Y + 1,
		},
		weight: 1,
	})
	queue = append(queue, &Node{
		pos: &Point{
			X: start.X,
			Y: start.Y - 1,
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
				X: node.pos.X + 1,
				Y: node.pos.Y,
			},
			weight: node.weight + 1,
		})
		queue = append(queue, &Node{
			pos: &Point{
				X: node.pos.X - 1,
				Y: node.pos.Y,
			},
			weight: node.weight + 1,
		})
		queue = append(queue, &Node{
			pos: &Point{
				X: node.pos.X,
				Y: node.pos.Y + 1,
			},
			weight: node.weight + 1,
		})
		queue = append(queue, &Node{
			pos: &Point{
				X: node.pos.X,
				Y: node.pos.Y - 1,
			},
			weight: node.weight + 1,
		})
		(*(moveState.BoardWeights))[node.pos.Y][node.pos.X] = node.weight
		moveState.MovesRemaining += 1
	}

	return moveState
}

func isValid(pos *Point, board *[][]int) bool {
	if isOutOfBound(pos, board) || isWall(pos, board) {
		//log.Printf("(%d, %d) is invalid", pos.X, pos.Y)
		return false
	} else {
		return true
	}
}

func isOutOfBound(pos *Point, board *[][]int) bool {
	if pos.Y >= len(*board) || pos.Y < 0 || pos.X < 0 || pos.X >= len((*board)[0]) {
		return true
	} else {
		return false
	}
}

func isWall(pos *Point, board *[][]int) bool {
	if (*board)[pos.Y][pos.X] == -1 {
		return true
	} else {
		return false
	}
}

func isVisited(pos *Point, board *[][]int) bool {
	if (*board)[pos.Y][pos.X] > 0 {
		//log.Printf("(%d, %d) has already been checked", pos.X, pos.Y)
		return true
	} else {
		return false
	}
}

func findAllPaths(head, dest *Point, gameState *[]*MoveState) *[]*[]*Point {
	paths := make([]*[]*Point, 4)
	paths[LEFT] = findPath(&Point{head.X - 1, head.Y}, dest, (*gameState)[LEFT])
	paths[RIGHT] = findPath(&Point{head.X + 1, head.Y}, dest, (*gameState)[RIGHT])
	paths[UP] = findPath(&Point{head.X, head.Y - 1}, dest, (*gameState)[UP])
	paths[DOWN] = findPath(&Point{head.X, head.Y + 1}, dest, (*gameState)[DOWN])
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
		if next.Y < len(*board)-1 && ((*board)[next.Y+1][next.X]-(*board)[next.Y][next.X] == -1 || (next.Y+1 == start.Y && next.X == start.X)) {
			next = &Point{
				X: next.X,
				Y: next.Y + 1,
			}
		} else if next.Y > 0 && ((*board)[next.Y-1][next.X]-(*board)[next.Y][next.X] == -1 || (next.Y-1 == start.Y && next.X == start.X)) {
			next = &Point{
				X: next.X,
				Y: next.Y - 1,
			}
		} else if next.X < len((*board)[0])-1 && ((*board)[next.Y][next.X+1]-(*board)[next.Y][next.X] == -1 || (next.Y == start.Y && next.X+1 == start.X)) {
			next = &Point{
				X: next.X + 1,
				Y: next.Y,
			}
		} else if next.X > 0 && ((*board)[next.Y][next.X-1]-(*board)[next.Y][next.X] == -1 || (next.Y == start.Y && next.X-1 == start.X)) {
			next = &Point{
				X: next.X - 1,
				Y: next.Y,
			}
		}

		path = append(path, next)
	}
	ReversePath(path)
	return &path
}

func ReversePath(array []*Point) {
	for i, j := 0, len(array)-1; i < j; i, j = i+1, j-1 {
		array[i], array[j] = array[j], array[i]
	}
}

func calculateSafeDirection(preferredDirs *[]int, bodyLength int, gameState *[]*MoveState, head *Point, body *[]Point) int {
	// log.Println((*gameState)[1].MovesRemaining, bodyLength)
	for i, dir := range *preferredDirs {
		if (*gameState)[dir] != nil && (*gameState)[dir].MovesRemaining >= bodyLength {
			// log.Println("aaaa", (*preferredDirs)[i])
			return (*preferredDirs)[i]
		}
	}

	maxSpaces := math.MinInt32
	bestDir := -1
	for i, moveState := range *gameState {
		if moveState == nil {
			continue
		}
		if moveState.MovesRemaining > maxSpaces {
			maxSpaces = moveState.MovesRemaining
			bestDir = i
		} else if moveState.MovesRemaining == maxSpaces {
			bestDir = -1
		}
	}
	if bestDir == -1 {
		nextMaxSpaces := math.MinInt32
		for i, moveState := range *gameState {
			if moveState == nil || moveState.MovesRemaining < maxSpaces {
				continue
			}
			//calculate max moves remaining for direction i
			remainingSpaces := findMaxRemainingSpaces(head, body, moveState.BoardWeights)
			if remainingSpaces > nextMaxSpaces {
				nextMaxSpaces = remainingSpaces
				bestDir = i
			}
		}
	}
	return bestDir
}

func findMaxRemainingSpaces(head *Point, body *[]Point, board *[][]int) int {
	states := make([]*MoveState, 4)
	left := &Point{head.X - 1, head.Y}
	right := &Point{head.X + 1, head.Y}
	down := &Point{head.X, head.Y + 1}
	up := &Point{head.X, head.Y - 1}

	states[LEFT] = createMoveState(left, body, board)

	states[RIGHT] = createMoveState(right, body, board)

	states[UP] = createMoveState(up, body, board)

	states[DOWN] = createMoveState(down, body, board)

	maxSpaces := math.MinInt32
	for _, state := range states {
		if state != nil && state.MovesRemaining > maxSpaces {
			maxSpaces = state.MovesRemaining
		}
	}
	//log.Println("max remaining spaces:", maxSpaces, states)
	return maxSpaces
}

func NewGameStartRequest(req *http.Request) (*GameStartRequest, error) {
	decoded := GameStartRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)
	return &decoded, err
}

//func CalculateDirectionFromPath(path *[]*Point) string {
//	if (*path)[1].X-(*path)[0].X == -1 {
//		return "left"
//	} else if (*path)[1].X-(*path)[0].X == 1 {
//		return "right"
//	} else if (*path)[1].Y-(*path)[0].Y == -1 {
//		return "up"
//	} else if (*path)[1].Y-(*path)[0].Y == 1 {
//		return "down"
//	} else {
//	    return "up" //TODO the default should be handled better
//	}
//}
