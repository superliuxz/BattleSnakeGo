package main

import (
	"log"
	"net/http"
	"os"
	"io"
)

func main() {
	fs := http.FileServer(http.Dir("static"))
	http.Handle("/", fs)

	http.HandleFunc("/derrick/start", handleStart)
	http.HandleFunc("/derrick/move", handleMove)

	port := os.Getenv("PORT")
	if port == "" {
		port = "9000"
	}
	f, err := os.Create("log.txt")
	if err != nil {
		panic(err)
	}
	mw := io.MultiWriter(f, os.Stdout)
	log.SetOutput(mw)
	log.Printf("Running server on port %s...\n", port)
	http.ListenAndServe(":"+port, nil)
}
