# 
# docker buildx build --tag keesverruijt/mayara-demo:latest .
#
# docker buildx build --platform linux/arm64,linux/amd64 --tag keesverruijt/mayara-demo:latest --push .
#

FROM rust:1.94-alpine AS builder
# For building openssl (which is needed for build.rs)
RUN apk add --no-cache perl g++ make curl

WORKDIR /usr/src/mayara-server
COPY build.rs Cargo.lock Cargo.toml ./
COPY src src/
COPY web web/
RUN cargo build --release

FROM alpine:latest
RUN apk add --no-cache tcpreplay
COPY --from=builder /usr/src/mayara-server/target/release/mayara-server /usr/local/bin/mayara-server
COPY demo/start.sh /
COPY demo/samples/halo_and_0183.pcap /
COPY demo/settings.json /root/.config/mayara/settings.json
CMD [ "/start.sh"]
