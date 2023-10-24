FROM python:3.9.18-slim-bookworm

RUN mkdir /home/app

WORKDIR /home/app

COPY ./app .

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        xz-utils \
        libexpat1 \
        curl \
        gnupg \
        libpq-dev \
        default-libmysqlclient-dev \
        unzip \
        libodbc1 \
        apt-transport-https \
        swig \
        # GIS libraries for GeoDjango (https://docs.djangoproject.com/en/3.2/ref/contrib/gis/install/geolibs/)
        binutils \
        libproj-dev \
        gdal-bin \
        libgdal-dev \
        python3-gdal \
        unixodbc-dev \
        libgssapi-krb5-2 \
    && rm -rf /var/lib/apt/lists/* 

RUN pip install --upgrade pip \
    && pip install gunicorn \
    && pip install debugpy \
    && pip install viztracer==0.15.6 \
    && pip install vizplugins==0.1.3 \
    && pip install orjson==3.8.1 \
    && if [ "%PYTHON_VERSION%" = "3.7" ] || [ "%PYTHON_VERSION%" = "3.8" ]; then curl -LO http://ftp.de.debian.org/debian/pool/main/libf/libffi/libffi6_3.2.1-9_amd64.deb \
    && dpkg -i libffi6_3.2.1-9_amd64.deb \
    && rm libffi6_3.2.1-9_amd64.deb; fi \
    && ln -s /opt/startupcmdgen/startupcmdgen /usr/local/bin/oryx \
    && apt-get update \
    && apt-get upgrade --assume-yes \
    && rm -rf /var/lib/apt/lists/*

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.5/install.sh | bash

RUN . ~/.nvm/nvm.sh

RUN nvm install 16

RUN cd /home/app/backend/approaches/checker/jslib \
    && npm install

RUN cd /home/app/frontend \
    && npm install

RUN cd /home/app/backend \
    && pip install -r requirements.txt

CMD ["python3", "-m", "gunicorn", "app:app"]

