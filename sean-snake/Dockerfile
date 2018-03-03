FROM golang:latest 
RUN mkdir /app 
ADD . /app/ 
WORKDIR /app 
RUN go build -o sean-snake . 
CMD ["/app/sean-snake"]