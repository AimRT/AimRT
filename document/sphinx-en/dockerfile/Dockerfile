FROM nginx:latest

ENV SERVER_ROOT=/usr/share/nginx/html

COPY nginx.conf /etc/nginx/conf.d/default.conf

ADD html.tar.gz /usr/share/nginx/html

WORKDIR /usr/share/nginx/html

CMD ["nginx", "-g", "daemon off;"]
