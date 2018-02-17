package main

import (
	"encoding/json"
	"fmt"
	"net/http"
	//"log"
	"bytes"
	"time"
	"log"
)

func respond(res http.ResponseWriter, obj interface{}) {
	res.Header().Set("Content-Type", "application/json")
	json.NewEncoder(res).Encode(obj)
}

func handleStart(res http.ResponseWriter, req *http.Request) {
	data, err := NewGameStartRequest(req)
	if err != nil {
		respond(res, GameStartResponse{
			Taunt:   toStringPointer("battlesnake-go!"),
			Color:   "#00FF00",
			Name:    fmt.Sprintf("%v (%vx%v)", data.GameId, data.Width, data.Height),
			HeadUrl: toStringPointer(fmt.Sprintf("%v://%v/static/head.png")),
		})
	}

	//scheme := "http"
	//if req.TLS != nil {
	//	scheme = "https"
	//}
	respond(res, GameStartResponse{
		Taunt:   toStringPointer("jsut another snk from the zoo..."),
		Color:   "#DA70D6",
		Name:    fmt.Sprintf("%v (%vx%v)", data.GameId, data.Width, data.Height),
		//HeadUrl: toStringPointer(fmt.Sprintf("%v://%v/static/head.png", scheme, req.Host)),
		HeadUrl: toStringPointer("https://marketplace.magento.com/media/catalog/product/cache/image" +
			"/750x360/e9c3970ab036de70892d86c6d221abfe/i/c/icon-256x256_2_1_1_2_1_1.png"),
	})
}

func handleMove(res http.ResponseWriter, req *http.Request) {
	var buffer bytes.Buffer
	start := time.Now()
	data, err := NewMoveRequest(req, &buffer)
	log.Printf("Took %s", time.Since(start))
	log.Println()
	if err != nil {
		respond(res, MoveResponse{
			Move:  "up",
			Taunt: toStringPointer("Oops, our logic failed"),
		})
		return
	}

	respond(res, MoveResponse{
		Move:  buffer.String(),
		Taunt: &data.You.Taunt,
	})
}
