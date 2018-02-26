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
	BoardWeights	*[][]int
	MovesRemaining	int
	You		  		*Snake
}

func simulateMove(moveState *MoveState, newHead *Point) bool {
	board := moveState.BoardWeights
	if isOutOfBound(newHead, board) {
		return false
	}
	newSnake := make([]Point, len(moveState.You.Body.Data))
	newSnake[0] = *newHead
	for i, body := range moveState.You.Body.Data {
		(*board)[body.Y][body.X] = 0
		if i+1 == len(moveState.You.Body.Data) {
			break
		}
		newSnake[i+1] = body
	}
	for i, body := range newSnake {
		if (*board)[body.Y][body.X] != 0 || i == 0 {
			continue
		}
		(*board)[body.Y][body.X] = - (len(newSnake)-i)
	}
	if isWall(newHead, board) {
		return false
	}
	(*board)[newHead.Y][newHead.X] = -len(newSnake)
	moveState.You.Body.Data = newSnake
	return true
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

	var closestFood *Point

	preferredFood := preferredFood(&head, &decoded.Food.Data, &board, &otherSnakes)
	if len(*preferredFood) >= 1 {
		// not using Pop in case we want to do something else
		closestFood = (*preferredFood)[0].value
	} else {
		closestFood = &decoded.Food.Data[0]
	}

	left := &Point{head.X - 1, head.Y}
	right := &Point{head.X + 1, head.Y}
	down := &Point{head.X, head.Y + 1}
	up := &Point{head.X, head.Y - 1}

	gameState[LEFT] = createMoveState(left, &decoded.You, &otherSnakes, &board)

	gameState[RIGHT] = createMoveState(right, &decoded.You, &otherSnakes, &board)

	gameState[UP] = createMoveState(up, &decoded.You, &otherSnakes, &board)

	gameState[DOWN] = createMoveState(down, &decoded.You, &otherSnakes, &board)

	paths := findAllPaths(&decoded.You.Body.Data[0], closestFood, &gameState)

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
				continue
			}
			if len(*path) < min {
				bestDir = dir
				min = len(*path)
			}
		}
		if bestDir == -1 {
			bestDir = availableDirs[0]
		}
		preferredDirs[i] = bestDir
		i++
		for j, dir := range availableDirs {
			if dir == bestDir {
				availableDirs = append(availableDirs[:j], availableDirs[j+1:]...)
				break
			}
		}
	}
	blockSnakeDirs := calculateKillMoves(&board, &head, &decoded.You, &otherSnakes)
	if len(blockSnakeDirs) > 0 {
		oldPreferredDirs := preferredDirs
		preferredDirs = blockSnakeDirs
		for _, dir := range oldPreferredDirs {
			inDirs := false
			for _, blockDir := range blockSnakeDirs {
				if dir == blockDir {
					inDirs = true
					break
				}
			}
			if !inDirs {
				preferredDirs = append(preferredDirs, dir)
			}
		}
	}
	bestDir := calculateSafeDirection(&preferredDirs, &board, &gameState, &decoded.You, &otherSnakes)
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

func calculateKillMoves(board *[][]int, head *Point, snake *Snake, snakes *[]Snake) []int {
	preferredDirs := make([]int, 0)
	for _, dir := range []int{LEFT,RIGHT,UP,DOWN} {
		moves := 0
		currentPos := head
		for moves < 4 {
			currentPos = nextPoint(currentPos, dir)
			if isOutOfBound(currentPos, board) || (*board)[currentPos.Y][currentPos.X] < 0 {
				currentPos = nextPoint(currentPos, (dir+2) % 4)
				break
			}
			moves++
		}
		if moves == 0 || moves == 4{
			continue
		}
		i := 0
		moveState := &MoveState {
			BoardWeights:   copyBoard(board),
			MovesRemaining: 0,
			You: copySnake(snake),
		}
		for i < moves {
			simulateMove(moveState, nextPoint(&moveState.You.Body.Data[0], dir))
			i++
		}
		blocksSnake := false
		for _, otherSnake := range *snakes {
			newSnakeState := &MoveState {
				BoardWeights:   copyBoard(moveState.BoardWeights),
				MovesRemaining: 0,
				You: copySnake(&otherSnake),
			}
			floodFillMoveState(&otherSnake.Body.Data[0], newSnakeState)
			oldSnakeState := &MoveState {
				BoardWeights:   copyBoard(board),
				MovesRemaining: 0,
				You: copySnake(&otherSnake),
			}
			floodFillMoveState(&otherSnake.Body.Data[0], oldSnakeState)
			if newSnakeState.MovesRemaining < oldSnakeState.MovesRemaining - moves && moves < (*oldSnakeState.BoardWeights)[currentPos.Y][currentPos.X] {
				log.Printf("With move %d snake had %d spaces but now has %d", dir, oldSnakeState.MovesRemaining, newSnakeState.MovesRemaining)
				blocksSnake = true
				break
			}

		}
		if blocksSnake {
			preferredDirs = append(preferredDirs, dir)
		}
	}
	return preferredDirs
}

func nextPoint(start *Point, dir int) *Point{
	switch dir {
		case LEFT: return &Point{start.X-1, start.Y}
		case RIGHT: return &Point{start.X+1, start.Y}
		case DOWN: return &Point{start.X, start.Y+1}
		case UP: return &Point{start.X, start.Y-1}
		default: return nil
	}
}

func longestPath(start, dest *Point, board *[][]int) int {
	var preferredDir []int
	xDiff := start.X - dest.X
	yDiff := start.Y - dest.Y
	bestX := -1
	bestY := -1
	directions := make([]*Point, 4)
	directions[LEFT] = &Point{start.X - 1, start.Y}
	directions[RIGHT] = &Point{start.X + 1, start.Y}
	directions[DOWN] = &Point{start.X, start.Y + 1}
	directions[UP] = &Point{start.X, start.Y - 1}
	if xDiff > 0 {
		bestX = RIGHT
	} else if xDiff < 0 {
		bestX = LEFT
	}

	if yDiff > 0 {
		bestY = DOWN
	} else if yDiff < 0 {
		bestY = UP
	}
	if bestY == -1 {
		preferredDir = []int{bestX, UP, DOWN, (bestX+2)%4}
	} else if bestX == -1 {
		preferredDir = []int{bestY, LEFT, RIGHT, (bestY+2)%4}
	} else {
		preferredDir = []int{bestX, bestY, (bestX+2)%4, (bestY+2)%4}
	}
	for _, dir := range preferredDir {
		if *directions[dir] == *dest {
			return dir
		}
		if  !isValid(directions[dir], board) {
			continue
		}
		if AStarSearch(directions[dir], dest, board) == nil {
			continue
		}

		return dir
	}
	return UP
}

func firstOpenSpace(board *[][]int, activeWalls *[]*Point) *Point {
	var bestSpace *Point
	bestValue := math.MinInt32
	for _, wall := range *activeWalls {
		if (*board)[wall.Y][wall.X] > bestValue {
			bestSpace = wall
			bestValue = (*board)[wall.Y][wall.X]
		}
	}
	return bestSpace
}

func preferredFood(head *Point, foods *[]Point, board *[][]int, snakes *[]Snake) *PriorityQueue {

	moveState := &MoveState {copyBoard(board), 0, nil}
	floodFillMoveState(head, moveState)

	snakeWeights := make([]*MoveState, len(*snakes))
	for i, snake := range *snakes {
		snakeMoveState := &MoveState {copyBoard(board), 0, nil}
		floodFillMoveState(&snake.Body.Data[0], snakeMoveState)
		snakeWeights[i] = snakeMoveState
	}

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	for _, food := range *foods {
		if (*moveState.BoardWeights)[food.Y][food.X] == 0 {
			continue
		}
		minWeight := math.MaxInt32
		for _, weights := range snakeWeights {
			snakeWeight := (*weights.BoardWeights)[food.Y][food.X]
			if snakeWeight == 0 {
				continue
			}
			if snakeWeight < minWeight  {
				minWeight = snakeWeight
			}
		}
		score := (*moveState.BoardWeights)[food.Y][food.X]
		if minWeight <= score {
			score = 50 + (score - minWeight)
		}
		heap.Push(&pq, &PriorityQueueNode{value: &Point{food.X,food.Y}, priority: float64(score)})
	}

	return &pq
}

func activeWalls(head *Point, board *[][]int) *[]*Point {
	walls := make([]*Point, 0)
	visited := make(map[Point]bool)
	var queue []*Point
	queue = append(queue, head)
	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]
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
			} else if (*board)[left.Y][left.X] < 0 && ((*board)[curr.Y][curr.X] == 0 || *curr == *head) {
				walls = append(walls, left)
			}
		}
		if !isOutOfBound(right, board) {
			if (*board)[right.Y][right.X] == 0 {
				queue = append(queue, right)
			} else if (*board)[right.Y][right.X] < 0 && ((*board)[curr.Y][curr.X] == 0 || *curr == *head) {
				walls = append(walls, right)
			}
		}
		if !isOutOfBound(up, board) {
			if (*board)[up.Y][up.X] == 0 {
				queue = append(queue, up)
			} else if (*board)[up.Y][up.X] < 0 && ((*board)[curr.Y][curr.X] == 0 || *curr == *head) {
				walls = append(walls, up)
			}
		}
		if !isOutOfBound(down, board) {
			if (*board)[down.Y][down.X] == 0 {
				queue = append(queue, down)
			} else if (*board)[down.Y][down.X] < 0 && ((*board)[curr.Y][curr.X] == 0 || *curr == *head) {
				walls = append(walls, down)
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
			if (*board)[y][x] < 0 {
				newBoard[y][x] = (*board)[y][x]
			}
		}
	}
	return &newBoard
}

func floodFillMoveState(start *Point, moveState *MoveState) {
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
}

func copySnake(snake *Snake) *Snake {
	return &Snake {
		Health: snake.Health,
		Id: snake.Id,
		Name: snake.Name,
		Taunt: snake.Taunt,
		Length: snake.Length,
		Body: snake.Body,
	}
}

func createMoveState(start *Point, snake *Snake, otherSnakes *[]Snake, originalBoard *[][]int) *MoveState {

	moveState := &MoveState{
		BoardWeights:   copyBoard(originalBoard),
		MovesRemaining: 0,
		You: copySnake(snake),
	}

	if !simulateMove(moveState, start) {
		return nil
	}

	floodFillMoveState(start, moveState)

	return moveState
}

func isValid(pos *Point, board *[][]int) bool {
	if isOutOfBound(pos, board) || isWall(pos, board) {
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
	if (*board)[pos.Y][pos.X] < 0 {
		return true
	} else {
		return false
	}
}

func isVisited(pos *Point, board *[][]int) bool {
	if (*board)[pos.Y][pos.X] > 0 {
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

func moveIsTrap(move *Point, board *[][]int, snakes *[]Snake) bool {
	nextMove := move
	lastMove := move
	for true {
		availableMoves := make([]*Point, 0)
		left := &Point{nextMove.X - 1, nextMove.Y}
		right := &Point{nextMove.X + 1, nextMove.Y}
		down := &Point{nextMove.X, nextMove.Y + 1}
		up := &Point{nextMove.X, nextMove.Y - 1}
		if isValid(left, board) && *left != *lastMove {
			availableMoves = append(availableMoves, left)
		}
		if isValid(right, board) && *right != *lastMove  {
			availableMoves = append(availableMoves, right)
		}
		if isValid(down, board) && *down != *lastMove  {
			availableMoves = append(availableMoves, down)
		}
		if isValid(up, board) && *up != *lastMove  {
			availableMoves = append(availableMoves, up)
		}
		if len(availableMoves) == 0 {
			return false //This is already a dead end, calculate safe path will try to save us
		}
		if len(availableMoves) > 1 {
			break
		}
		lastMove = nextMove
		nextMove = availableMoves[0]
	}
	if nextMove == move {
		return false
	}

	distanceToExit := (*board)[nextMove.Y][nextMove.X]
	for _, snake := range *snakes {
		snakeMoveState := &MoveState{
			BoardWeights:   copyBoard(board),
			MovesRemaining: 0,
			You: nil,
		}
		floodFillMoveState(&snake.Body.Data[0], snakeMoveState)
		if (*snakeMoveState.BoardWeights)[nextMove.Y][nextMove.X] == 0 {
			continue
		}
		if (*snakeMoveState.BoardWeights)[nextMove.Y][nextMove.X] < distanceToExit {
			return true
		}
	}
	return false
}

func moveCouldHitSnakeHead(snakes *[]Snake, you *Snake, move *Point) bool {
	bodyLength := len(you.Body.Data)
	for _, snake := range *snakes {
		if len(snake.Body.Data) < bodyLength {
			continue
		}
		snakeHead := snake.Body.Data[0]
		if snakeHead.X+1 == move.X && snakeHead.Y == move.Y {
			return true
		}
		if snakeHead.X-1 == move.X && snakeHead.Y == move.Y {
			return true
		}
		if snakeHead.X == move.X && snakeHead.Y+1 == move.Y {
			return true
		}
		if snakeHead.X == move.X && snakeHead.Y-1 == move.Y {
			return true
		}
	}
	return false
}

func calculateSafeDirection(preferredDirs *[]int, board *[][]int, gameState *[]*MoveState, you *Snake, snakes *[]Snake) int {
	body := &you.Body.Data
	head := &(*body)[0]
	bodyLength := len(*body)
	for i, dir := range *preferredDirs {
		if (*gameState)[dir] == nil {
			continue
		}
		var start *Point
		switch dir {
			case LEFT: start = &Point{head.X-1, head.Y}
			case RIGHT: start = &Point{head.X+1, head.Y}
			case DOWN: start = &Point{head.X, head.Y+1}
			case UP: start = &Point{head.X, head.Y-1}
		}

		if moveCouldHitSnakeHead(snakes, you, start) {
			continue
		}

		if moveIsTrap(start, (*gameState)[dir].BoardWeights, snakes) {
			continue
		}

		if (*gameState)[dir].MovesRemaining >= bodyLength-1 {
			if findMaxRemainingSpaces(start, you, snakes, (*gameState)[dir].BoardWeights) >= bodyLength-1 {
				return (*preferredDirs)[i]
			}
		}
	}

	activeWalls := activeWalls(head, board)
	openSpace := firstOpenSpace(board, activeWalls)
	return longestPath(head, openSpace, board)
}

func findMaxRemainingSpaces(head *Point, snake *Snake, otherSnakes *[]Snake, board *[][]int) int {
	states := make([]*MoveState, 4)
	left := &Point{head.X - 1, head.Y}
	right := &Point{head.X + 1, head.Y}
	down := &Point{head.X, head.Y + 1}
	up := &Point{head.X, head.Y - 1}

	states[LEFT] = createMoveState(left, snake, otherSnakes, board)

	states[RIGHT] = createMoveState(right, snake, otherSnakes, board)

	states[UP] = createMoveState(up, snake, otherSnakes, board)

	states[DOWN] = createMoveState(down, snake, otherSnakes, board)

	maxSpaces := math.MinInt32
	for _, state := range states {
		if state != nil && state.MovesRemaining > maxSpaces {
			maxSpaces = state.MovesRemaining
		}
	}
	return maxSpaces
}

func NewGameStartRequest(req *http.Request) (*GameStartRequest, error) {
	decoded := GameStartRequest{}
	err := json.NewDecoder(req.Body).Decode(&decoded)
	return &decoded, err
}

func manhattanDistance(x, y *Point) float64 {
	return math.Abs(float64(x.X-y.X)) + math.Abs(float64(x.Y-y.Y))
}

func AStarSearch(start, dest *Point, board *[][]int) *[]Point {
	visited := make(map[Point]bool)

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)

	head := &PriorityQueueNode{
		value:    start,
		priority: manhattanDistance(start, dest) + 0,
		parent:   nil,
	}

	heap.Push(&pq, head)

	for pq.Len() > 0 {
		curr := heap.Pop(&pq).(*PriorityQueueNode)
		if visited[*curr.value] {
			continue
		}

		if *curr.value == *dest {
			return CalculatePath(curr)
		}

		visited[*curr.value] = true

		for _, neighbour := range *Expand(curr, dest, board) {
			if !visited[*neighbour.value] {
				heap.Push(&pq, neighbour)
			}
		}
	}
	return nil
}

func Expand(curr *PriorityQueueNode, dest *Point, board *[][]int) *[]*PriorityQueueNode {
	successor := make([]*PriorityQueueNode, 0)
	pathCost := curr.priority
	var next *Point

	if (curr.value.Y+1 < len(*board) && (*board)[curr.value.Y+1][curr.value.X] > -1) || (curr.value.Y+1 == dest.Y && curr.value.X == dest.X) {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y + 1,
		}
		successor = append(successor, &PriorityQueueNode{
			value:    next,
			priority: pathCost + 1 + manhattanDistance(next, dest),
			parent:   curr,
		})
	}

	if (curr.value.Y-1 >= 0 && (*board)[curr.value.Y-1][curr.value.X] > -1) || (curr.value.Y-1 == dest.Y && curr.value.X == dest.X) {

		next = &Point{
			X: curr.value.X,
			Y: curr.value.Y - 1,
		}
		successor = append(successor, &PriorityQueueNode{
			value:    next,
			priority: pathCost + 1 + manhattanDistance(next, dest),
			parent:   curr,
		})
	}

	if (curr.value.X+1 < len((*board)[0]) && (*board)[curr.value.Y][curr.value.X+1] > -1) || (curr.value.Y == dest.Y && curr.value.X+1 == dest.X) {

		next = &Point{
			X: curr.value.X + 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &PriorityQueueNode{
			value:    next,
			priority: pathCost + 1 + manhattanDistance(next, dest),
			parent:   curr,
		})
	}

	if (curr.value.X-1 >= 0 && (*board)[curr.value.Y][curr.value.X-1] > -1) || (curr.value.Y == dest.Y && curr.value.X-1 == dest.X) {

		next = &Point{
			X: curr.value.X - 1,
			Y: curr.value.Y,
		}
		successor = append(successor, &PriorityQueueNode{
			value:    next,
			priority: pathCost + 1 + manhattanDistance(next, dest),
			parent:   curr,
		})
	}

	return &successor
}

func CalculatePath(curr *PriorityQueueNode) *[]Point {

	pathSize := 0
	dest := curr
	for curr != nil {
		pathSize += 1
		curr = curr.parent
	}

	path := make([]Point, pathSize)
	curr = dest
	for curr != nil {
		pathSize -= 1
		path[pathSize] = *curr.value
		curr = curr.parent
	}

	return &path
}
