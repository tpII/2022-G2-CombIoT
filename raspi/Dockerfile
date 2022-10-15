FROM python:3.8.6-alpine3.12

ENV VERSION=3.8.6

RUN apk add openjdk8 wget gcc g++ \
    && wget -O /tmp/maven.tar.gz https://dlcdn.apache.org/maven/maven-3/$VERSION/binaries/apache-maven-$VERSION-bin.tar.gz --no-check-certificate \
	&& tar -zxvf /tmp/maven.tar.gz -C /etc/ \
	&& mv /etc/apache-maven-$VERSION /etc/maven \
    && pip install pyzxing

ENV PATH=/etc/maven/bin:$PATH