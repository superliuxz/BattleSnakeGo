# Battlesnake

FROM amazonlinux:2017.09

WORKDIR /deploy/app/

# Install platform dependencies
RUN yum -y update
RUN yum install -y nginx make python36 python36-pip python27-pip golang
RUN pip install supervisor
RUN yum clean all

# Configure Nginx
RUN mkdir -p /run/nginx /etc/nginx/sites-available /etc/nginx/sites-enabled \
    && rm -f /etc/nginx/sites-enabled/default
COPY conf/etc/nginx/sites-available/app.conf /etc/nginx/sites-available/
RUN ln -s /etc/nginx/sites-available/app.conf /etc/nginx/sites-enabled/app.conf
COPY conf/etc/nginx/nginx.conf /etc/nginx/nginx.conf
RUN rm -rf /var/log/nginx/*
RUN mkdir /usr/share/nginx/logs && touch /usr/share/nginx/logs/error.log

# Configure supervisor
RUN touch /var/log/messages
RUN mkdir -p /etc/supervisor /etc/supervisor/conf.d /var/log/supervisor
COPY conf/etc/supervisor/supervisord.conf /etc/supervisor/supervisord.conf

# Create snake binary
COPY commands.go commands.go
COPY data.go data.go
COPY main.go main.go
COPY util.go util.go
RUN go build -o battlesnake_18
RUN chmod 777 battlesnake_18

# Start processes
CMD ["/usr/local/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
