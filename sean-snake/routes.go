package main

import (
	"fmt"
	"log"
	"math/rand"
	"net/http"
	"time"
)

func Start(res http.ResponseWriter, req *http.Request) {
	log.Print("START REQUEST")

	data, err := NewStartRequest(req)
	if err != nil {
		log.Printf("Bad start request: %v", err)
	}
	dump(data)

	respond(res, StartResponse{
		Taunt:          "We slitherin'!",
		Color:          "#a300cc",
		HeadURL:        "https://i.ytimg.com/vi/qbZbu2QkLRM/hqdefault.jpg",
		Name:           "Sean da Snek",
		HeadType:       HEAD_SHADES,
		TailType:       TAIL_SMALL_RATTLE,
		SecondaryColor: "#7a0099",
	})
}

func Move(res http.ResponseWriter, req *http.Request) {
	log.Printf("MOVE REQUEST")

	data, err := NewMoveRequest(req)

	if err != nil {
		log.Printf("Bad move request: %v", err)
	}
	dump(data)

	var possibleDirections = GetPossibleDirections(data)

	fmt.Println(possibleDirections)

	r := rand.New(rand.NewSource(time.Now().UnixNano()))

	respond(res, MoveResponse{
		Move: possibleDirections[r.Intn(len(possibleDirections))],
	})
}

func GetPossibleDirections(data *MoveRequest) []string {

	possibleDirections := []string{}

	myHead := data.You.Body[0]

	leftPoint := Point{X: (myHead.X - 1), Y: myHead.Y}
	rightPoint := Point{X: (myHead.X + 1), Y: myHead.Y}
	upPoint := Point{X: myHead.X, Y: (myHead.Y - 1)}
	downPoint := Point{X: myHead.X, Y: (myHead.Y + 1)}

	if isPointValid(&leftPoint, data) {
		possibleDirections = append(possibleDirections, "left")
	}
	if isPointValid(&rightPoint, data) {
		possibleDirections = append(possibleDirections, "right")
	}
	if isPointValid(&upPoint, data) {
		possibleDirections = append(possibleDirections, "up")
	}
	if isPointValid(&downPoint, data) {
		possibleDirections = append(possibleDirections, "down")
	}

	return possibleDirections
}

func isPointValid(coordinates *Point, data *MoveRequest) bool {
	if coordinates.X < 0 {
		return false
	}
	if coordinates.Y < 0 {
		return false
	}
	if coordinates.X > data.Width-1 {
		return false
	}
	if coordinates.Y > data.Height-1 {
		return false
	}
	for _, snake := range data.Snakes {
		for _, snakebody := range snake.Body {
			if coordinates.X == snakebody.X && coordinates.Y == snakebody.Y {
				return false
			}
		}
	}
	return true
}
